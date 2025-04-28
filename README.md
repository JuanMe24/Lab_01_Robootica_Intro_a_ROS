# Lab_01_Robootica_Intro_a_ROS
## Proyecto: Control de Tortuga en Turtlesim

## Descripción general

Este proyecto consiste en controlar una tortuga en el simulador `turtlesim` usando Python y ROS2.  
Se implementaron dos modos principales de control:

- **Control manual** usando las flechas del teclado (↑ ↓ ← →) para mover la tortuga.
- **Dibujo automático** de letras basadas en las iniciales de los nombres de los integrantes del equipo, pulsando teclas específicas (en minúscula).

El objetivo principal fue desarrollar un único script que gestionara completamente el movimiento de la tortuga, sin depender de `turtle_teleop_key`.

---

## Procedimiento realizado

1. **Control manual**:
   - Se detectan las teclas de flechas:
     - ↑ (flecha arriba): avanzar hacia adelante.
     - ↓ (flecha abajo): retroceder.
     - → (flecha derecha): girar hacia la derecha.
     - ← (flecha izquierda): girar hacia la izquierda.
   - Cada flecha genera un `Twist` publicado en `/turtle1/cmd_vel`.

2. **Dibujo automático**:
   - Pulsando letras minúsculas específicas se dibujan letras sobre el entorno:
     - `m` → Dibuja la letra **M**.
     - `a` → Dibuja la letra **A**.
     - `c` → Dibuja la letra **C**.
     - `j` → Dibuja la letra **J**.
     - `d` → Dibuja la letra **D**.
     - `h` → Dibuja la letra **H**.
   - Cada función de dibujo utiliza comandos de movimiento y rotación (`move`) para crear las formas deseadas.

---

## Funcionamiento general

- Ejecutar el script `move_turtle.py`.
- La tortuga puede ser controlada inmediatamente:
  - Usar flechas para moverse manualmente.
  - Pulsar una de las letras (`m`, `a`, `c`, `j`, `d`, `h`) para que la tortuga dibuje automáticamente la letra correspondiente.
- Pulsar `q` para salir del programa de forma segura.

---

## Código principal (estructura básica)

```python
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
        thread = threading.Thread(target=self.key_listener)
        thread.start()

    def key_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:
                key = sys.stdin.read(1)
                msg = Twist()
                if key == '\x1b':  # Detectar flechas
                    if sys.stdin.read(1) == '[':
                        arrow_key = sys.stdin.read(1)
                        if arrow_key == 'A':
                            msg.linear.x = 2.0
                        elif arrow_key == 'B':
                            msg.linear.x = -2.0
                        elif arrow_key == 'C':
                            msg.angular.z = -2.0
                        elif arrow_key == 'D':
                            msg.angular.z = 2.0
                        self.publisher.publish(msg)
                elif key == 'm':
                    self.draw_m()
                elif key == 'a':
                    self.draw_a()
                elif key == 'c':
                    self.draw_c()
                elif key == 'j':
                    self.draw_j()
                elif key == 'd':
                    self.draw_d()
                elif key == 'h':
                    self.draw_h()
                elif key == 'q':
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

    # Ejemplo de funciones de dibujo (se definen similarmente todas las letras)
    def draw_m(self):
        # Código para dibujar la letra M
        pass

    def draw_a(self):
        # Código para dibujar la letra A
        pass

    def draw_c(self):
        # Código para dibujar la letra C
        pass

    def draw_j(self):
        # Código para dibujar la letra J
        pass

    def draw_d(self):
        # Código para dibujar la letra D
        pass

    def draw_h(self):
        # Código para dibujar la letra H
        pass

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
