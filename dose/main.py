import rclpy
import os
from . import pump
from rclpy.node import Node
from std_msgs.msg import String
import time
import datetime

PIN_IN1 = int(os.getenv('PIN_IN1',22))
PIN_IN2 = int(os.getenv('PIN_IN2',23))
PIN_EN = int(os.getenv('PIN_EN',23))

TARGET_PH = float(os.getenv('TARGET_PH', 5.8))

DOSE_INTERVAL = int(os.getenv('DOSE_INTERVAL', 5))

DOSE_MIN_MINUTE_DIFF = int(os.getenv('DOSE_MIN_MINUTE_DIFF', 5.5))

class Dose(Node):

    def __init__(self):
        super().__init__('dose')
        self.get_logger().info('Startig dose pin in1="%d", in2="%d", en="%d" and target ph of "%d"' % (PIN_IN1, PIN_IN1, PIN_EN, TARGET_PH))
        self.pump = pump.Pump(PIN_IN1, PIN_IN2, PIN_EN)
        self.subscription = self.create_subscription(
            String,
            'ph',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        self.get_logger().info('Getting ph value of: "%d"' % msg.data)
        
        ph = msg.data

        if ph <= TARGET_PH:
            return 
        
        if hasattr(Dose, 'last_run'):
            last_run_difference = datetime.now() - self.last_run
            minutes = last_run_difference.total_seconds() / 60  
            if minutes < DOSE_MIN_MINUTE_DIFF:
                return
            
        self.pump.flow()
        time.sleep(DOSE_INTERVAL)
        self.pump.stop()
        self.last_run = datetime.now()
        self.get_logger().info('Staring. Last run was: "%s"' % self.last_run.strftime('%Y-%m-%d %H:%M:%S'))                
        

def main(args=None):
    rclpy.init(args=args)

    pump_subscriber = Dose()

    rclpy.spin(pump_subscriber)

    pump_subscriber.pump.shutdown()
    pump_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()