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

PIN_PUMP2_E = int(os.getenv('PIN_PUMP2_IN1',0))
PIN_PUMP2_M = int(os.getenv('PIN_PUMP2_IN2',0))

TARGET_METRIC = os.getenv('TARGET_METRIC', 'ph')
TARGET_VALUE = float(os.getenv('TARGET_VALUE', 7.0))

DOSE_INTERVAL = int(os.getenv('DOSE_INTERVAL', 5))

DOSE_MIN_MINUTE_DIFF = int(os.getenv('DOSE_MIN_MINUTE_DIFF', 60))

DAY = 'day'
NIGHT = 'night'

class Dose(Node):

    def __init__(self):
        super().__init__('dose')
        self.get_logger().info('Startig dose pin in1="%d", in2="%d", en="%d" and target ph of "%d"' % (PIN_IN1, PIN_IN2, PIN_EN, TARGET_PH))
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

        if value <= TARGET_VALUE:
            self.get_logger().info('No dose - value less than target')
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
        
        self.pump1.flow()
        if self.pump2:
            self.pump2.flow()

        time.sleep(DOSE_INTERVAL)
        self.pump.stop()
        if self.pump2:
            self.pump2.stop()
        
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