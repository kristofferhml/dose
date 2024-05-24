import RPi.GPIO as GPIO

class Pump():
    def __init__(self, PIN_IN1, PIN_IN2, PIN_EN):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PIN_IN1, GPIO.OUT)
        GPIO.setup(PIN_IN2, GPIO.OUT)
        GPIO.setup(PIN_EN, GPIO.OUT)
        
        GPIO.output(PIN_IN1, GPIO.HIGH)
        GPIO.output(PIN_IN2, GPIO.HIGH)
        self.p = GPIO.PWM(PIN_EN, 1000)
        self.p.start(0)

        self.current = GPIO.HIGH
        self.CONTROL_PIN = PIN_IN1
        
    def flow(self):
        if self.current == GPIO.LOW:
            return
       
        GPIO.output(self.CONTROL_PIN, GPIO.LOW)
        self.current = GPIO.LOW

    def stop(self):
        if self.current == GPIO.HIGH:
            return
        
        GPIO.output(self.CONTROL_PIN, GPIO.HIGH)
        self.current = GPIO.HIGH
       
    def shutdown(self):
        GPIO.cleanup()