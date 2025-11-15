echo "Ejemplo para llamar al servicio /darp_service"

ros2 service call /darp_service multi_robot_planning_rms_msgs/srv/DarpPetition "{
    min_x: -5,
    max_x: 5,
    min_y: -5,
    max_y: 5,  
    visualization: true, 
    initial_positions: [  
      {x: 0.0, y: 0.0},
      {x: 5.0, y: 5.0},
      {x: -3.0, y: -3.0}
    ],
    obstacle_points: [
      {x: 1.0, y: 1.0},
      {x: 2.0, y: 2.0},
      {x: -1.0, y: -1.0}
    ]
  }"