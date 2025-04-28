import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time

class TurtlesimLetterDrawer(Node):
    def __init__(self):
        super().__init__('turtlesim_letter_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.running = True

        # hilo para teclas
        thread = threading.Thread(target=self.key_listener)
        thread.start()

    def key_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:
                key = sys.stdin.read(1)
                if key == 'm':
                    self.get_logger().info("Dibujando la letra M")
                    self.draw_m()
                elif key == 'a':
                    self.get_logger().info("Dibujando la letra A")
                    self.draw_a()
                elif key == 'h':
                    self.get_logger().info("Dibujando la letra H")
                    self.draw_h()
                elif key == 'j':
                    self.get_logger().info("Dibujando la letra J")
                    self.draw_j()
                elif key == 'd':
                    self.get_logger().info("Dibujando la letra D")
                    self.draw_d()
                elif key == 'q':
                    self.get_logger().info("Saliendo...")
                    self.running = False
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        time.sleep(duration)
        self.stop()

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)
        time.sleep(0.2)

    def draw_m(self):    #hilo para dibujar la letra M
        self.move(0.0, 1.57, 1.0)    
        self.move(2.0, 0.0, 1.5)    
        self.move(0.0, -2.355, 1.0)  
        self.move(1.0, 0.0, 1.0)    
        self.move(0.0, 1.57, 1.0)    
        self.move(1.0, 0.0, 1.0)    
        self.move(0.0, -2.35, 1.0)    
        self.move(2.0, 0.0, 1.5)
        self.move(0.0, 1.57, 1.0) 

    def draw_a(self):   #hilo para dibujar la letra A
        self.move(0.0, 1.046, 1.0)    
        self.move(2.0, 0.0, 1.5)    
        self.move(0.0, -2.093, 1.0)    
        self.move(2.0, 0.0, 2.0)   
        self.move(0.0, 3.15, 1.0)   
        self.move(1.0, 0.0, 1.0)    
        self.move(0.0, 1.046, 1.0)
        self.move(1.0, 0.0, 1.0) 
        self.move(0.0, 3.14, 1.0)  

    def draw_h(self):   #hilo para dibujar la letra H
        self.move(0.0, 1.57, 1.0)    
        self.move(2.0, 0.0, 1.5)    
        self.move(0.0, 3.15, 1.0)     
        self.move(1.0, 0.0, 1.5)       
        self.move(0.0, 1.57, 1.0)    
        self.move(1.0, 0.0, 1.5) 
        self.move(0.0, 1.57, 1.0)  
        self.move(1.0, 0.0, 1.5) 
        self.move(0.0, 3.15, 1.0)
        self.move(2.0, 0.0, 1.5) 
        self.move(0.0, 1.57, 1.0)

    def draw_j(self):   #hilo para dibujar la letra J
        self.move(0.0, -1.57, 1.0)
        self.move(1.0, 2.0, 2.0)
        self.move(0.8, 1.15, 1.8)
        self.move(2.0, 0.0, 1.5)

    def draw_d(self):   #hilo para dibujar la letra D
        self.move(0.0, 1.57, 1.0)    
        self.move(3.0, 0.0, 1.5)
        self.move(0.0, -1.57, 1.0)
        self.move(1.0, 0.0, 1.0)
        msg = Twist()
        msg.linear.x = 1.5
        msg.angular.z = -1.55
        self.publisher.publish(msg)
        time.sleep(2)
        self.stop()
        self.move(1.0, 0.0, 0.5)
        msg = Twist()
        msg.linear.x = 1.5
        msg.angular.z = -1.55
        self.publisher.publish(msg)
        time.sleep(2)
        self.stop()
        self.move(1.0, 0.0, 1.0)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimLetterDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()