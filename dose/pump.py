import RPi.GPIO as GPIO

class Pump():
    def __init__(self, PIN_E, PIN_M):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PIN_E, GPIO.OUT)
        GPIO.setup(PIN_M, GPIO.OUT)
            
        GPIO.output(PIN_E, GPIO.LOW)
        GPIO.output(PIN_M, GPIO.LOW)
        self.p = GPIO.PWM(PIN_M, 1000)
        self.p.start(0)

        self.current = GPIO.LOW
        self.control_pin = PIN_E
        
    def flow(self):
        if self.current == GPIO.HIGH:
            return
       
        GPIO.output(self.control_pin, GPIO.HIGH)   
        self.p.ChangeDutyCycle(80)
        
        self.current = GPIO.HIGH

    def stop(self):
        if self.current == GPIO.LOW:
            return
        
        GPIO.output(self.control_pin, GPIO.LOW)
        self.p.ChangeDutyCycle(0)
        self.current = GPIO.LOW
       
    def shutdown(self):
        GPIO.cleanup()