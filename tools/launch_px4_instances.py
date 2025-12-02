#!/usr/bin/env python3

"""
Launch n PX4 instances with initial positions from config file
Usage: ./launch_px4_instances.py [num_vehicles]
"""

import os
import sys
import yaml
import subprocess
import time
import signal
import termios
from pathlib import Path


def load_config(config_path):
    """Load and parse YAML configuration file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def expand_path(path):
    """Expand ~ in paths."""
    return os.path.expanduser(path)


def restore_terminal():
    """Restore terminal to normal state."""
    try:
        # Try to restore terminal settings
        if sys.stdin.isatty():
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, saved_terminal_state)
    except (termios.error, NameError):
        # Terminal state wasn't saved or restore failed, try to enable echo
        try:
            if sys.stdin.isatty():
                attrs = termios.tcgetattr(sys.stdin.fileno())
                attrs[3] |= termios.ECHO  # Enable echo
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, attrs)
        except termios.error:
            pass  # Ignore if we can't restore


def cleanup_processes(processes):
    """Terminate all spawned processes."""
    print("\nShutting down processes...")
    sys.stdout.flush()
    sys.stderr.flush()
    
    for process in processes:
        if process and process.poll() is None:  # Process is still running
            try:
                process.terminate()
            except Exception as e:
                print(f"Error terminating process {process.pid}: {e}")
    
    # Wait a bit for graceful shutdown
    time.sleep(2)
    
    # Force kill if still running
    for process in processes:
        if process and process.poll() is None:
            try:
                process.kill()
            except Exception as e:
                print(f"Error killing process {process.pid}: {e}")
    
    # Restore terminal state
    restore_terminal()
    sys.stdout.flush()
    sys.stderr.flush()


def main():
    # Save terminal state for restoration on exit
    global saved_terminal_state
    try:
        if sys.stdin.isatty():
            saved_terminal_state = termios.tcgetattr(sys.stdin.fileno())
    except (termios.error, AttributeError):
        saved_terminal_state = None
    
    # Get script directory and config path
    script_dir = Path(__file__).parent
    config_file = script_dir.parent / 'config' / 'px4_vehicles.yaml'
    
    if not config_file.exists():
        print(f"Error: Config file not found: {config_file}", file=sys.stderr)
        sys.exit(1)
    
    # Load configuration
    config = load_config(config_file)
    px4_config = config.get('px4_config', {})
    agent_config = config.get('agent_config', {})
    positions = config.get('initial_positions', [])
    
    # Extract configuration values
    num_positions = len(positions)
    autostart = px4_config.get('autostart', 4001)
    model = px4_config.get('model', 'gz_x500_mod')
    world = px4_config.get('world', 'warehouse')
    px4_binary = expand_path(px4_config.get('px4_binary_path', 
        '~/shared_volume/PX4-Autopilot/build/px4_sitl_default/bin/px4'))
    agent_port = agent_config.get('port', 8888)
    agent_protocol = agent_config.get('protocol', 'udp4')
    
    # Determine number of vehicles
    num_vehicles = int(sys.argv[1]) if len(sys.argv) > 1 else num_positions
    if num_vehicles > num_positions:
        print(f"Warning: Requested {num_vehicles} vehicles but only {num_positions} positions available")
        num_vehicles = num_positions
    
    # Check if PX4 binary exists
    if not os.path.isfile(px4_binary):
        print(f"Error: PX4 binary not found: {px4_binary}", file=sys.stderr)
        sys.exit(1)
    
    # Change to PX4 directory
    px4_dir = os.path.dirname(px4_binary)
    os.chdir(px4_dir)
    
    print(f"Launching {num_vehicles} PX4 instances (Model: {model}, World: {world})")
    
    # Launch PX4 instances
    px4_processes = []
    for i in range(1, num_vehicles + 1):
        pos = positions[i - 1]
        pose_str = f"{pos['x']},{pos['y']}"
        
        env = os.environ.copy()
        env['PX4_SYS_AUTOSTART'] = str(autostart)
        env['PX4_SIM_MODEL'] = model
        env['PX4_GZ_WORLD'] = world
        
        if i == 1:
            # First instance creates the world
            env['PX4_GZ_MODEL_POSE'] = pose_str
            cmd = ['./px4', '-i', str(i)]
            log_file = f'/tmp/px4_instance_{i}.log'
        else:
            # Subsequent instances use STANDALONE mode with position
            env['PX4_GZ_STANDALONE'] = '1'
            env['PX4_GZ_MODEL_POSE'] = pose_str
            cmd = ['./px4', '-i', str(i)]
            log_file = f'/tmp/px4_instance_{i}.log'
        
        with open(log_file, 'w') as log:
            process = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
            px4_processes.append(process)
            print(f"  Instance {i}: PID {process.pid} at ({pose_str})")
        
        # Wait after first instance, shorter delay for others
        time.sleep(10 if i == 1 else 2)
    
    time.sleep(5)
    
    # Launch MicroXRCEAgent if not running
    agent_process = None
    try:
        result = subprocess.run(['pgrep', '-f', 'MicroXRCEAgent'], 
                              capture_output=True, check=False)
        if result.returncode == 0:
            print("MicroXRCEAgent already running")
        else:
            log_file = '/tmp/microxrce_agent.log'
            with open(log_file, 'w') as log:
                agent_process = subprocess.Popen(['MicroXRCEAgent', agent_protocol, '-p', str(agent_port)],
                                                stdout=log, stderr=subprocess.STDOUT)
                print(f"MicroXRCEAgent started (PID: {agent_process.pid})")
    except FileNotFoundError:
        print("Warning: pgrep not found, skipping agent check")

    # Store all processes for cleanup
    all_processes = px4_processes + ([agent_process] if agent_process else [])
    
    # Set up signal handler for Ctrl+C
    def signal_handler(sig, frame):
        cleanup_processes(all_processes)
        restore_terminal()
        sys.stdout.flush()
        sys.stderr.flush()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Print summary
    print("Press Ctrl+C to stop all processes\n")
    
    # Wait for all processes (or until interrupted)
    try:
        while True:
            # Check if any process has died
            for process in all_processes:
                if process and process.poll() is not None:
                    print(f"Process {process.pid} has exited")
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_processes(all_processes)
        restore_terminal()
        sys.stdout.flush()
        sys.stderr.flush()


if __name__ == '__main__':
    main()

