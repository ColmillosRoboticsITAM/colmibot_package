import rclpy
import threading

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

left_right_buttons = 0.0
up_down_buttons = 0.0
x_button = 0
enable_movement = True


def topic_qos():
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE
    )


def joy_callback(joy_msg: Joy):
    global left_right_buttons
    global up_down_buttons
    global x_button
    left_right_buttons = joy_msg.axes[6]
    up_down_buttons = joy_msg.axes[7]
    x_button = joy_msg.buttons[0]


def main():
    global left_right_buttons
    global up_down_buttons
    global x_button
    linear = 0.0
    angular = 0.0

    print("TELEOPERATION OF COLMIBOT WITH A JOYSTICK")
    
    rclpy.init()
    teleop_colmibot_node = rclpy.create_node('teleop_colmibot_joystick')
    
    scale_angular_param = teleop_colmibot_node.declare_parameter("scale_angular", 2.0)
    scale_linear_param = teleop_colmibot_node.declare_parameter("scale_linear", 2.0)
    
    qos = topic_qos()
    twist_pub = teleop_colmibot_node.create_publisher(Twist, "colmibot/cmd_vel", qos)

    joy_sub = teleop_colmibot_node.create_subscription(Joy, "joy", joy_callback, qos)

    spinner = threading.Thread(target=rclpy.spin, args=(teleop_colmibot_node,))
    spinner.start()

    twist_msg = Twist()

    while True:
        try:
            scale_angular_param = teleop_colmibot_node.get_parameter("scale_angular").get_parameter_value().double_value
            scale_linear_param = teleop_colmibot_node.get_parameter("scale_linear").get_parameter_value().double_value
            
            # Mover hacia adelante
            if (up_down_buttons >= 1.0):
                linear = 1.0
                enable_movement = True
            # Mover hacia atrás
            elif (up_down_buttons <= -1.0):
                linear = -1.0
                enable_movement = True
            # Rotaar hacia la izquierda
            elif (left_right_buttons >= 1.0):
                angular = -1.0
                enable_movement = True
            # Rotar hacia la derecha
            elif (left_right_buttons <= -1.0):
                angular = 1.0
                enable_movement = True
            elif (x_button == 1 and enable_movement == True):
                linear = 0.0
                angular = 0.0
                enable_movement = False
                twist_msg.angular.z = scale_angular_param * angular
                twist_msg.linear.x = scale_linear_param * linear
                twist_pub.publish(twist_msg)

            if ((linear != 0.0) or (angular != 0.0)):
                twist_msg.angular.z = scale_angular_param * angular
                twist_msg.linear.x = scale_linear_param * linear
                twist_pub.publish(twist_msg)
                linear = 0.0
                angular = 0.0

            left_right_buttons = 0.0
            up_down_buttons = 0.0
        
        except KeyboardInterrupt:
            print("Keyboard interrupt (Ctrl-c)")
            break

if __name__ == '__main__':
    main()