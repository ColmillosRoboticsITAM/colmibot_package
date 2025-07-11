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

## Controlar el colmibot con el tecaldo

Asumiendo que la ESP32 ya está conectada por cable serial a la computadora, correr
los siguientes comandos en una terminal:
```
cd colmibot_ws
source install/local_setup.bash
ros2 run colmibot teleop_keyboard
```

## Controlar el colmibot con un joystick
Antes de correr el nodo para controlar el colmibot con un joystick se debe
instalar el paquete de microros.

También se debe instalar el paquete de joy_linux:
```
sudo apt-get install ros-jazzy-joy-linux
```

Asumiendo que la ESP32 ya está conectada por cable serial a la computadora y el
control de playstation 4 ejecutar los siguientes nodos:

En una terminal correr el nodo de este paquete:
```
cd colmibot_ws
source install/local_setup.bash
ros2 run colmibot teleop_joystick
```

En otra terminal correr el nodo de micro_ros, verificar que el puerto serial es ttyUSB0:
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

En otra terminal correr el nodo que lee el joystick:
```
ros2 run joy_linux joy_linux_node
```

## Referencias:

Los siguientes paquetes de ROS2 se utilizaron como referencia:
- https://github.com/ros/ros_tutorials/blob/jazzy/turtlesim/tutorials/teleop_turtle_key.cpp
- https://github.com/ros2/teleop_twist_keyboard
- https://github.com/ros-teleop/teleop_twist_keyboard
- https://github.com/ros-teleop/teleop_twist_joy