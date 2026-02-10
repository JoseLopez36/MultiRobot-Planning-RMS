echo "EJEMPLO 1: Grid básico con 3 UAVs"

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
    ],
    traversed_points: []
  }"

echo ""
echo "EJEMPLO 2: Grid con sub-regiones"
echo ""
echo "Grid (6x5):"
echo "     Col: 0 1 2 3 4 5"
echo "Fila 0:  0 0 1 0 0 X   (UAV1)"
echo "Fila 1:  X 0 1 T T T   (UAV0)"
echo "Fila 2:  T 0 1 0 0 0"
echo "Fila 3:  T T 1 0 1 1"
echo "Fila 4:  0 0 1 0 1 0"
echo ""
echo "Leyenda: X=UAV, 0=Libre, 1=Obstáculo, T=Recorrido"
echo ""

# Descomentar para ejecutar:
ros2 service call /darp_service multi_robot_planning_rms_msgs/srv/DarpPetition "{
  min_x: 0,
  max_x: 5,
  min_y: 0,
  max_y: 4,
  visualization: true,
  initial_positions: [
    {x: 0.0, y: 1.0},
    {x: 5.0, y: 0.0}
  ],
  obstacle_points: [
    {x: 2.0, y: 0.0},
    {x: 2.0, y: 1.0},
    {x: 2.0, y: 2.0},
    {x: 2.0, y: 3.0},
    {x: 2.0, y: 4.0},
    {x: 4.0, y: 3.0},
    {x: 5.0, y: 3.0},
    {x: 4.0, y: 4.0}
  ],
  traversed_points: [
    {x: 3.0, y: 1.0},
    {x: 4.0, y: 1.0},
    {x: 5.0, y: 1.0},
    {x: 0.0, y: 2.0},
    {x: 0.0, y: 3.0},
    {x: 1.0, y: 3.0}
  ]
}"

echo ""
echo "EJEMPLO 3: Grid 9x9 con muro vertical y sub-regiones simétricas"
echo ""
echo "Grid (9x9), coordenadas mundo:"
echo "     Col: -4 -3 -2 -1  0  1  2  3  4"
echo "y= 4:  X  0  0  0  1  0  0  0  0"
echo "y= 3:  0  0  0  0  1  0  0  0  0"
echo "y= 2:  T  T  T  T  1  0  0  0  0"
echo "y= 1:  0  0  0  0  1  0  0  0  0"
echo "y= 0:  0  0  0  0  1* 0  0  0  0"
echo "y=-1:  0  0  0  0  1  0  0  0  0"
echo "y=-2:  0  0  0  0  1  T  T  T  T"
echo "y=-3:  0  0  0  0  1  0  0  0  0"
echo "y=-4:  0  0  0  0  1  0  0  0  X"
echo ""
echo "Leyenda: X=UAV, 0=Libre, 1=Obstáculo, T=Recorrido, 1*=Obstáculo clave"
echo "Quitando el obstáculo de (0,0) [marcado con 1*], las dos mitades se conectan"
echo "y la solución cambia completamente."
echo ""
echo "Sub-regiones esperadas:"
echo "  Zona Izquierda:"
echo "    Sub A (UAV0): y=4,3 cols -4..-1 = 8 celdas"
echo "    Sub B (sin UAV): y=1,0,-1,-2,-3,-4 cols -4..-1 = 24 celdas"
echo "  Zona Derecha:"
echo "    Sub C (sin UAV): y=4,3,2,1,0,-1 cols 1..4 = 24 celdas"
echo "    Sub D (UAV1): y=-3,-4 cols 1..4 = 8 celdas"
echo ""

ros2 service call /darp_service multi_robot_planning_rms_msgs/srv/DarpPetition "{
  min_x: -4,
  max_x: 4,
  min_y: -4,
  max_y: 4,
  visualization: true,
  initial_positions: [
    {x: -4.0, y: 4.0},
    {x: 4.0, y: -4.0}
  ],
  obstacle_points: [
    {x: 0.0, y: 4.0},
    {x: 0.0, y: 3.0},
    {x: 0.0, y: 2.0},
    {x: 0.0, y: 1.0},
    {x: 0.0, y: 0.0},
    {x: 0.0, y: -1.0},
    {x: 0.0, y: -2.0},
    {x: 0.0, y: -3.0},
    {x: 0.0, y: -4.0}
  ],
  traversed_points: [
    {x: -4.0, y: 2.0},
    {x: -3.0, y: 2.0},
    {x: -2.0, y: 2.0},
    {x: -1.0, y: 2.0},
    {x: 1.0, y: -2.0},
    {x: 2.0, y: -2.0},
    {x: 3.0, y: -2.0},
    {x: 4.0, y: -2.0}
  ]
}"