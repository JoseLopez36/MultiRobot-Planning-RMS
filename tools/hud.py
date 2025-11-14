import sys
import subprocess
import tempfile
from PySide6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout
from PySide6.QtGui import QPainter, QColor, QPen, QFont
from PySide6.QtCore import Qt, QPoint

class MapWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("background-color: #001F3F;")
        self.points = []  # Lista de tuplas: (x, y, tipo, id)
        self.current_type = "UAV"
        self.uav_counter = 1
        self.ugv_counter = 1

    def set_vehicle_type(self, tipo):
        self.current_type = tipo

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            # Limitar a 3 UAVs seleccionados
            if self.current_type == "UAV":
                num_uavs = sum(1 for p in self.points if p['tipo'] == "UAV")
                if num_uavs >= 3:
                    print("Solo se pueden seleccionar 3 UAVs.")
                    return
            pos = event.position()
            center_x = self.width() / 2
            center_y = self.height() / 2
            x_relative = int((pos.x() - center_x) * 0.1)  # Escala: 10 píxeles = 1 metros
            y_relative = int((center_y - pos.y()) * 0.1)  # Escala: 10 píxeles = 1 metros
            if self.current_type == "UAV":
                vehicle_id = self.uav_counter
                self.uav_counter += 1
            else:
                vehicle_id = self.ugv_counter
                self.ugv_counter += 1
            self.points.append({
                'x_screen': int(pos.x()),
                'y_screen': int(pos.y()),
                'x_rel': x_relative,
                'y_rel': y_relative,
                'tipo': self.current_type,
                'id': vehicle_id
            })
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        
        # --- GRID DE FONDO ---
        grid_pen = QPen(QColor(0, 0, 255, 120))  # Azul claro, semitransparente
        grid_pen.setWidth(1)
        painter.setPen(grid_pen)
        grid_spacing = 10  # píxeles por "10 metros"
        width = self.width()
        height = self.height()
        center_x = width // 2
        center_y = height // 2

        # Líneas verticales
        x = center_x
        while x < width:
            painter.drawLine(x, 0, x, height)
            x += grid_spacing
        x = center_x - grid_spacing
        while x > 0:
            painter.drawLine(x, 0, x, height)
            x -= grid_spacing

        # Líneas horizontales
        y = center_y
        while y < height:
            painter.drawLine(0, y, width, y)
            y += grid_spacing
        y = center_y - grid_spacing
        while y > 0:
            painter.drawLine(0, y, width, y)
            y -= grid_spacing

        # --- CRUZ CENTRAL ---
        pen_cross = QPen(QColor(255, 255, 0))  # Amarillo intenso
        pen_cross.setWidth(2)
        painter.setPen(pen_cross)
        painter.drawLine(0, int(center_y), width, int(center_y))
        painter.drawLine(int(center_x), 0, int(center_x), height)

        # Etiqueta (0,0)
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        painter.drawText(QPoint(int(center_x + 10), int(center_y - 10)), "(0,0)")

        # --- VEHÍCULOS ---
        for point in self.points:
            x = point['x_screen']
            y = point['y_screen']
            tipo = point['tipo']
            vehicle_id = point['id']
            pen = QPen(Qt.cyan if tipo == "UAV" else QColor(0, 150, 255))
            pen.setWidth(2)
            painter.setPen(pen)
            painter.setBrush(QColor(0, 255, 255) if tipo == "UAV" else QColor(0, 150, 255))
            painter.drawEllipse(QPoint(x, y), 6, 6)
            painter.setFont(QFont("Arial", 9))
            label = f"{tipo}_{vehicle_id}"
            painter.drawText(QPoint(x + 10, y - 10), label)

class HUD(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HUD - UAV/UGV Deployment")
        self.setStyleSheet("background-color: #001530; color: cyan; font-size: 14px;")
        self.map = MapWidget()

        # Botones para seleccionar tipo
        self.btn_uav = QPushButton("UAV")
        self.btn_ugv = QPushButton("UGV")
        self.btn_uav.setStyleSheet("background-color: #003366; color: cyan; font-weight: bold;")
        self.btn_ugv.setStyleSheet("background-color: #003366; color: cyan; font-weight: bold;")
        self.btn_uav.clicked.connect(lambda: self.map.set_vehicle_type("UAV"))
        self.btn_ugv.clicked.connect(lambda: self.map.set_vehicle_type("UGV"))

        # Botón para enviar lote
        self.btn_send = QPushButton("Enviar Lote")
        self.btn_send.setStyleSheet("background-color: #006633; color: white; font-weight: bold;")
        self.btn_send.clicked.connect(self.send_lote)

        hlayout = QHBoxLayout()
        hlayout.addWidget(self.btn_uav)
        hlayout.addWidget(self.btn_ugv)
        hlayout.addWidget(self.btn_send)

        layout = QVBoxLayout()
        layout.addLayout(hlayout)
        layout.addWidget(self.map)
        self.setLayout(layout)

    def send_lote(self):
        if not self.map.points:
            print("No hay vehículos para enviar.")
            return

        # Crear fichero temporal con las posiciones relativas al centro (0,0) e ID
        with tempfile.NamedTemporaryFile(mode="w+", delete=False, suffix=".txt") as f:
            for point in self.map.points:
                tipo = point['tipo']
                vehicle_id = point['id']
                x_rel = point['x_rel']
                y_rel = point['y_rel']
                f.write(f"{tipo} {vehicle_id} {x_rel} {y_rel}\n")
            f.flush()
            
            print(f"Enviando {len(self.map.points)} vehículos al publisher...")

            # Ejecutar el publisher y leer su salida
            proc = subprocess.Popen(
                ["./UAVPositionPublisher", f.name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )

            # Mostrar logs en tiempo real
            for line in proc.stdout:
                print(line, end="")

            proc.wait()
            if proc.returncode == 0:
                print("Lote enviado correctamente.")
            else:
                print(f"Error: el publisher terminó con código {proc.returncode}")
                for line in proc.stderr:
                    print(line, end="")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    hud = HUD()
    hud.resize(800, 600)
    hud.show()
    sys.exit(app.exec())
