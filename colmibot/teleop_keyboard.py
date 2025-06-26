import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist

import threading
import sys, select, termios, tty


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def get_key(settings):
	tty.setraw(sys.stdin.fileno())
    # La función se queda esperando hasta que reciba un caracter desde la terminal,
    # es decir, hasta que el usario presione una de las teclas
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
    
    # Con esta función limpiamos los caracteres que estén en los buffers de
    # entrada y salida de la terminal
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def main():
    running_node = True
    linear = 0.0
    angular = 0.0

    # Antes de ejecutar el programa se debe de guardar los parámetros de la
    # terminal donde se está ejecutando este programa
    settings = save_terminal_settings()
    
    rclpy.init()
    node = Node("teleop_colmibot_keyboard")
    
    # Con estos parámetros modificamos el valor escalar del ángulo y del
    # movimiento lineal
    node.declare_parameter("scale_angular", 2.0)
    node.declare_parameter("scale_linear", 2.0)

    # Establece la calidad de los mensajes publicados en el topico
    qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    twist_pub = node.create_publisher(Twist, "colmibot/cmd_vel", qos)

    # Se corre el nodo en un hilo por separado para que en el programa principal
    # se reciban los caracteres del teclado
    threading.Thread(target=rclpy.spin, args=(node, ), daemon=True).start()

    twist_msg = Twist()

    print("Reading from keyboard")
    print("----------------------------")
    print("Use wasd keys to move and rotate the turtle.")
    print("Use x to stop the movement.")
    print("'q' to quit")

    while(running_node == True):
        
        chr = get_key(settings)
        
        # Rotar a la izquierda
        if (chr == 'a'):
            angular = 1.0
            linear = 0.0
        # Rotar a la derecha
        elif (chr == 'd'):
            angular = -1.0
            linear = 0.0
        # Mover hacia adelante
        elif (chr == 'w'):
            angular = 0.0
            linear = 1.0
        # Mover hacia atrás
        elif (chr == 's'):
            angular = 0.0
            linear = -1.0
        elif (chr == 'q'):
            print("Terminate teleop_colmibot node")
            running_node = False
        # Pausar movimiento
        elif (chr == 'x'):
            angular = 0.0
            linear = 0.0
        
        scale_angular_param = node.get_parameter("scale_angular").get_parameter_value().double_value
        scale_linear_param = node.get_parameter("scale_linear").get_parameter_value().double_value
        
        if (running_node == True):
            twist_msg.angular.z = scale_angular_param * angular
            twist_msg.linear.x = scale_linear_param * linear
            twist_pub.publish(twist_msg)
    
    # Recuperar los párametros previos y aplicarlos a la terminal
    restore_terminal_settings(settings)

    rclpy.shutdown()

if __name__ == '__main__':
    main()