from time import time, sleep
#import spidev
import RPi.GPIO as gpio


class testbed_test():
    def __init__(self):
        
        self.button_pin = 24

        self.motor_pwm = 4 # pin 7
        self.motor_in1 = 21 # pin 
        self.motor_in2 = 27 # pin 
        self.motor_en = 13
        
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        
        gpio.setup(self.button_pin, gpio.IN, pull_up_down = gpio.PUD_DOWN)
        gpio.setup(self.motor_pwm, gpio.OUT)
        gpio.setup(self.motor_in1, gpio.OUT)
        gpio.setup(self.motor_in2, gpio.OUT)
        gpio.setup(self.motor_en, gpio.OUT)

        gpio.output(self.motor_en, gpio.HIGH)
        gpio.output(self.motor_in1, gpio.LOW)
        gpio.output(self.motor_in2, gpio.LOW)
        self.p = gpio.PWM(self.motor_pwm, 1000)
        self.p.start(50)

    def button_test(self):
        i = 0
        while True:
            
            if gpio.input(self.button_pin) == gpio.HIGH:
                print("{} button was pressed!!!!!!!!!!!".format(i))
                i += 1
            # sleep(.01)

    def motor(self):
        
        
        # while True:
            
        #     gpio.output(self.motor_in1, gpio.LOW)
        #     gpio.output(self.motor_in2, gpio.HIGH)

    
        gpio.output(self.motor_in1, gpio.LOW)
        gpio.output(self.motor_in2, gpio.HIGH)
        sleep(2)

        gpio.output(self.motor_in1, gpio.LOW)
        gpio.output(self.motor_in2, gpio.LOW)
        gpio.cleanup()



if __name__ == '__main__':
        

    test_num = input("""
    1) button
    2) motor
    3) stepper motor

    What do you want to test? (enter the number)
    """)


    test = testbed_test()

    if test_num == 1:

        test.button()

    elif test_num == 2:
        test.motor()

    else:
        print("not implemented yet")
