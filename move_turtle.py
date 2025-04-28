import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Variable de control de movimiento
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Hilo para leer las teclas
        self.key_listener_thread = threading.Thread(target=self.key_listener)
        self.key_listener_thread.daemon = True
        self.key_listener_thread.start()

        # Temporizador para mover la tortuga
        self.timer = self.create_timer(0.1, self.move_turtle)

    def key_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == '\x1b':  # Si detectamos ESC o flecha
                    key2 = sys.stdin.read(1)
                    if key2 == '[':
                        key3 = sys.stdin.read(1)
                        if key3 == 'A':  # Flecha arriba
                            self.linear_velocity = 1.7
                            self.angular_velocity = 0.0
                        elif key3 == 'B':  # Flecha abajo
                            self.linear_velocity = -1.7
                            self.angular_velocity = 0.0
                        elif key3 == 'C':  # Flecha derecha
                            self.linear_velocity = 0.0
                            self.angular_velocity = -2.0
                        elif key3 == 'D':  # Flecha izquierda
                            self.linear_velocity = 0.0
                            self.angular_velocity = 2.0
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
