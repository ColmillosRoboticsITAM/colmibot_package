# colmibot_package

Este paquete de ROS2 incluye los nodos necesarios para controlar el colmibot del ITAM.

## Pasos para instalar y compilar el paquete

1. Crear el *workspace* y descargar el paquete:
```
mkdir -p colmibot_ws/src
cd colmibot_ws/src
git clone https://github.com/ColmillosRoboticsITAM/colmibot_package.git
```

2. Compilar el paquete:
```
# Para compilar el paquete por primera vez:

cd colmibot_ws
colcon build

# Si se tuvieran más paquete dentro del workspace se puede compilar solo este
# paquete con el siguiente comando:

colcon build --packages-select colmibot
```

## Referencias:

Los siguientes paquetes de ROS2 se utilizaron como referencia:
https://github.com/ros/ros_tutorials/blob/jazzy/turtlesim/tutorials/teleop_turtle_key.cpp
https://github.com/ros2/teleop_twist_keyboard
https://github.com/ros-teleop/teleop_twist_keyboard