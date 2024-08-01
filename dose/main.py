import rclpy
import os
from . import pump
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
import time
from datetime import datetime

PIN_PUMP1_E = int(os.getenv('PIN_PUMP1_E',0))
PIN_PUMP1_M = int(os.getenv('PIN_PUMP1_M',0))

PIN_PUMP2_E = int(os.getenv('PIN_PUMP2_E',0))
PIN_PUMP2_M = int(os.getenv('PIN_PUMP2_M',0))

TARGET_METRIC = os.getenv('TARGET_METRIC', 'ph')
TARGET_VALUE = float(os.getenv('TARGET_VALUE', 7.0))
TARGET_DIRECTION = os.getenv('TARGET_DIRECTION', 'down')

DOSE_INTERVAL = int(os.getenv('DOSE_INTERVAL', 5))

DOSE_MIN_MINUTE_DIFF = int(os.getenv('DOSE_MIN_MINUTE_DIFF', 60))

DAY = 'day'
NIGHT = 'night'

class Dose(Node):

    def __init__(self):
        super().__init__('dose')
        self.get_logger().info('Startig dose pin e="%d", im="%d" taget metric="%s" and target value of "%d"' % (PIN_PUMP1_E, PIN_PUMP1_M, TARGET_METRIC, TARGET_VALUE))
        self.pump1 = pump.Pump(PIN_PUMP1_E, PIN_PUMP1_M)
        if PIN_PUMP2_E > 0:
            self.pump2 = pump.Pump(PIN_PUMP2_E, PIN_PUMP2_M)
       
        self.schedule = NIGHT

        self.subscription = self.create_subscription(
            Float32,
            TARGET_METRIC,
            self.listener_callback,
            10)
        
        self.subscription_day = self.create_subscription(
            String,
            'day',
            self.day_listener_callback,
            10)
    
    def day_listener_callback(self, msg):
        self.get_logger().info('Getting day value of: "%s"' % msg.data)
        self.schedule = msg.data
        
    def listener_callback(self, msg):
        self.get_logger().info('Getting value of: "%.2f"' % msg.data)
        
        if self.schedule == NIGHT:
            self.get_logger().info('No dose - schedule is night')
            return

        value = msg.data

        if TARGET_DIRECTION == "down" and value <= TARGET_VALUE:
            self.get_logger().info('No dose - value already less than target')
            return 
        
        if TARGET_DIRECTION == "up" and value >= TARGET_VALUE:
            self.get_logger().info('No dose - value already above target')
            return 
        
        try:
            last_run_difference = datetime.now() - self.last_run
            minutes = last_run_difference.total_seconds() / 60  
            self.get_logger().info('Minutes since last run: "%d"' % minutes)
            if minutes < DOSE_MIN_MINUTE_DIFF:
                self.get_logger().info('No dose - too close to last dose')
                return
        
        except AttributeError:
            self.get_logger().info('First run. Setting last_run')
            self.last_run = datetime.now()
        
        try:
            self.pump1.flow()
            self.pump2.flow()
        except AttributeError:
            self.get_logger().info('Only one pump attached')
        
        time.sleep(DOSE_INTERVAL)

        try:
            self.pump1.stop()
            self.pump2.stop()

        except AttributeError:
            self.get_logger().info('Only one pump attached')

        self.get_logger().info('Dose given. Last run was: "%s"' % self.last_run.strftime('%Y-%m-%d %H:%M:%S'))                
        self.last_run = datetime.now()
        

def main(args=None):
    rclpy.init(args=args)

    pump_subscriber = Dose()

    rclpy.spin(pump_subscriber)

    pump_subscriber.pump.shutdown()
    pump_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()