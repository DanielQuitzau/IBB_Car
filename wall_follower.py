import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# Ultraschall-Pins
TRIG = 23
ECHO = 24

# IR-Sensor rechts
IR_RIGHT = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(IR_RIGHT, GPIO.IN)

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.publisher = self.create_publisher(String, 'cmd_topic', 10)
        self.timer = self.create_timer(0.5, self.loop)
        self.get_logger().info("Wall follower started")

    def measure_distance(self):
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
        duration = pulse_end - pulse_start
        distance = duration * 34300 / 2
        return distance

    def loop(self):
        msg = String()
        dist = self.measure_distance()
        ir_right = GPIO.input(IR_RIGHT)

        if dist < 100:          # < 1 Meter
            msg.data = "S"
        else:
            if ir_right == 0:   # kein Hindernis rechts
                msg.data = "R"  # leicht nach rechts
            else:
                msg.data = "L"  # Wand erkannt → nach links

        self.publisher.publish(msg)
        self.get_logger().info(f"Dist: {dist:.1f} cm | IR: {ir_right} | CMD: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    GPIO.cleanup()

if __name__ == '__main__':
    main()
