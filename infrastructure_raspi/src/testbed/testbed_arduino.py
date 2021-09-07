#!/usr/bin/env python

#from os import DirEntry
from time import time, sleep
import RPi.GPIO as gpio
from stepper_motor import StepperMotor
import sys
import smbus2 as smbus#,smbus2

class Testbed():  # this is a test

    def __init__(self):
        
        # Slave Addresses
        self.I2C_SLAVE_ADDRESS = 15 #0x0b ou 11
        self.I2C_SLAVE2_ADDRESS = 14
        # Variables for moving cone up and down
        self.reset_cone_pul = 16 # pin
        self.reset_cone_dir = 20  # pin
        self.reset_cone_en = 12  # pin  (HIGH to Enable / LOW to Disable)

        self.reset_cone_speed = 0.00001 # default value

        
        self.cone_limit_switch = 17  # pin
        
        self.lift_time_limit = 3.0  # seconds
        self.lower_time_limit = 2.0  # seconds

        #contact plates on the cone - like a button
        self.cone_button = 14  # pin


        # Variables for spooling in/out the cable
        self.reset_cable_pul = 19 # pin Green wire
        self.reset_cable_dir = 6 # pin 31 Red wire
        self.reset_cable_en = 5 # pin 29 Blue wire (HIGH to Enable / LOW to Disable)
        self.reset_cable_speed = 0.00001  # default value

        self.spool_out_time_limit = 4.5  # seconds
        self.spool_in_time_limit = 20  # seconds

        
        # hall effect sensor for rotating table
        self.hall_effect  = 15  # pin

        # Variables for turntable/encoder wheel
        self.turntable_motor_pwm = 4 # pin 7
        self.turntable_motor_in1 = 21 # pin 
        self.turntable_motor_in2 = 27 # pin 
        self.turntable_motor_en = 13  

        # Variables for comunication with arduino for turntable encoder

        # Setting up the pins
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        gpio.setup(self.reset_cone_pul, gpio.OUT)
        gpio.setup(self.reset_cone_dir, gpio.OUT)
        gpio.setup(self.reset_cone_en, gpio.OUT)

        gpio.setup(self.reset_cable_pul, gpio.OUT)
        gpio.setup(self.reset_cable_dir, gpio.OUT)
        gpio.setup(self.reset_cable_en, gpio.OUT)

        # setting up turntable pins
        gpio.setup(self.turntable_motor_pwm, gpio.OUT)
        gpio.setup(self.turntable_motor_in1, gpio.OUT)
        gpio.setup(self.turntable_motor_in2, gpio.OUT)
        gpio.setup(self.turntable_motor_en, gpio.OUT)

        gpio.output(self.turntable_motor_en, gpio.HIGH)
        gpio.output(self.turntable_motor_in1, gpio.LOW)
        gpio.output(self.turntable_motor_in2, gpio.LOW)
        self.turntable_pwm = gpio.PWM(self.turntable_motor_pwm, 1000)
        self.turntable_pwm.start(50)

        gpio.setup(self.cone_limit_switch, gpio.IN, pull_up_down = gpio.PUD_DOWN)
        gpio.setup(self.hall_effect, gpio.IN)#, pull_up_down = gpio.PUD_DOWN)
        gpio.setup(self.cone_button, gpio.IN, pull_up_down = gpio.PUD_DOWN)

        # sets up the stepper motors
        self.reset_cone_motor = StepperMotor(self.reset_cone_pul, self.reset_cone_dir, self.reset_cone_en, self.reset_cone_speed)
        self.reset_cable_motor = StepperMotor(self.reset_cable_pul, self.reset_cable_dir, self.reset_cable_en, self.reset_cable_speed)
    def send_transmission(self,val,address):

        while True:
                    try:
                        sleep(0.01)
                        self.I2Cbus.write_i2c_block_data(address, 0x00, [val])
                        sleep(0.01)
                        break
                    except:
                        sleep(.05)
                        self.I2Cbus.write_i2c_block_data(address, 0x00, [val])
                        sleep(.05)
    
    def read_transmission(self,address):
        while True:
                try:
                    sleep(.001)
                    data=self.I2Cbus.read_byte_data(address,1)
                    return data
                except:
                    sleep(.001)

    def testbed_reset(self, angle=None):

        self.cone_reset_up()
        self.cable_reset_spool_in()
        sleep(1)
        self.cable_reset_spool_out(self.spool_out_time_limit)
        self.cone_reset_down()
        # self.cable_reset_spool_out()
        self.turntable_reset_home()
        #self.turntable_move_angle(356)


    def cone_reset_up(self, time_duration=None):
        self.I2Cbus = smbus.SMBus(1)
        with smbus.SMBus(1) as I2Cbus:
            if time_duration == None:
                time_duration = self.lift_time_limit
            start_time = time()
            lift_time = 0
            self.send_transmission(3,self.I2C_SLAVE_ADDRESS)
            while True:
                # button = gpio.input(self.cone_limit_switch)
                button = self.read_transmission(self.I2C_SLAVE_ADDRESS)
                if lift_time >= self.lift_time_limit or button == 1:
                    if button == 1:  
                        print("button was pressed")
                    else:
                        print("time ran out")
                    break
                self.reset_cone_motor.move_for(0.01, self.reset_cone_motor.CCW)
                lift_time = time() - start_time

    def cone_reset_down(self, time_duration=None):  # look at switching to steps moved
        if time_duration == None:
            time_duration = self.lower_time_limit
        start_time = time()
        lower_time = 0
        while True:
            if lower_time >= time_duration:
                break
            self.reset_cone_motor.move_for(0.01, self.reset_cone_motor.CW)
            lower_time = time() - start_time

    def cable_reset_spool_in(self):
        self.I2Cbus = smbus.SMBus(1)
        with smbus.SMBus(1) as I2Cbus:
            start_time = time()
            spool_in_time = 0
            self.send_transmission(4,self.I2C_SLAVE_ADDRESS)
            button_val = 0
            while True:
                sleep(.01)
                button_val = self.read_transmission(self.I2C_SLAVE_ADDRESS)
                #button_val = self.read_transmission(self.I2C_SLAVE_ADDRESS)
                if spool_in_time >= self.spool_in_time_limit or button_val == 1:
                    if button_val == 1:
                        print("button was pressed")
                    break
                self.reset_cable_motor.move_for(0.025, self.reset_cable_motor.CCW)  # check rotations
                spool_in_time = time() - start_time        

    def cable_reset_spool_out(self, var):
        start_time = time()
        spool_out_time = 0
        while True:
            if spool_out_time >= var:
                break
            self.reset_cable_motor.move_for(0.025, self.reset_cable_motor.CW)  # check rotations
            spool_out_time = time() - start_time
    
    def turntable_reset_home(self):
        self.I2Cbus = smbus.SMBus(1)
        with smbus.SMBus(1) as I2Cbus:
            delay = 0
            self.send_transmission(5,self.I2C_SLAVE_ADDRESS)
            sleep(.001)
            hall_effect = self.read_transmission(self.I2C_SLAVE_ADDRESS)
            if hall_effect == 0:  # ensures that it goes to the proper home orientation.
                delay = 2
            gpio.output(self.turntable_motor_in1, gpio.LOW)
            gpio.output(self.turntable_motor_in2, gpio.HIGH)
            
            while True:
                hall_effect = self.read_transmission(self.I2C_SLAVE_ADDRESS)
                if hall_effect == 0:
                    gpio.output(self.turntable_motor_in1, gpio.LOW)
                    gpio.output(self.turntable_motor_in2, gpio.LOW)
                    print("magnet detected")
                    break
                sleep(0.001)

    def turntable_move_angle(self, goal_angle=20):      
        self.I2Cbus = smbus.SMBus(1)
        with smbus.SMBus(1) as I2Cbus:  
            self.send_transmission(2,self.I2C_SLAVE_ADDRESS)
            print("Made it 1")
            self.read_transmission(self.I2C_SLAVE_ADDRESS)
            print("Made it 2")
            self.send_transmission(6, self.I2C_SLAVE_ADDRESS)
            print("Made it 3")
            sleep(0.0001)
            gpio.output(self.turntable_motor_in1, gpio.LOW)
            gpio.output(self.turntable_motor_in2, gpio.HIGH)
            counter = 0
            while True:
                    try:                    
                        first_byte =self.I2Cbus.read_byte_data(self.I2C_SLAVE_ADDRESS,1)
                        sleep(.001)
                        second_byte =self.I2Cbus.read_byte_data(self.I2C_SLAVE_ADDRESS,1)
                        encoder_value = (first_byte<< 8) + (second_byte)
                        print("recieve from slave:")
                        print(encoder_value)
                        sleep(.001)
                        if encoder_value >= goal_angle:
                            gpio.output(self.turntable_motor_in1, gpio.LOW)
                            gpio.output(self.turntable_motor_in2, gpio.LOW)
                            break
                    except:
                        print("remote i/o error")
                        self.send_transmission(8, self.I2C_SLAVE_ADDRESS)
                        while True:
                            try:
                                self.I2Cbus.read_byte_data(self.I2C_SLAVE_ADDRESS,1)
                                break
                            except:
                                print("remote i/o error")
                                sleep(.001)
                        while True:
                            try:
                                sleep(0.01)
                                self.I2Cbus.write_i2c_block_data(self.I2C_SLAVE_ADDRESS, 0x00, [6])
                                sleep(0.01)
                                break
                            except:
                                sleep(.001)
                                self.I2Cbus.write_i2c_block_data(self.I2C_SLAVE_ADDRESS, 0x00, [6])
                                sleep(.001)
                        sleep(.1)
            self.send_transmission(7,self.I2C_SLAVE_ADDRESS)
            self.read_transmission(self.I2C_SLAVE_ADDRESS)
            gpio.cleanup()

    def object_swap(self):
        self.I2Cbus = smbus.SMBus(1)
        with smbus.SMBus(1) as I2Cbus: 
            return_value = 0
            print("Made it 1")
            self.send_transmission(2,self.I2C_SLAVE2_ADDRESS)
            print("Made it 2")
            return_value = self.read_transmission(self.I2C_SLAVE2_ADDRESS)
            print("Made it 3")
            self.send_transmission(3,self.I2C_SLAVE2_ADDRESS)
            self.send_transmission(3,self.I2C_SLAVE2_ADDRESS)

            while True:
                sleep(0.1) 
                self.send_transmission(3,self.I2C_SLAVE2_ADDRESS)
                return_value = self.read_transmission(self.I2C_SLAVE2_ADDRESS)
                print(return_value)
                if return_value == 3:
                    break
            print("Sending Lower Reset Code")
            reset_testbed.cone_reset_up()
            reset_testbed.cable_reset_spool_in()
            reset_testbed.cone_reset_up()
            reset_testbed.cable_reset_spool_out(.75)

            self.send_transmission(4,self.I2C_SLAVE2_ADDRESS)
            return_value = self.read_transmission(self.I2C_SLAVE2_ADDRESS)
            sleep(4) 

            while True:
                sleep(0.1) 
                self.send_transmission(5,self.I2C_SLAVE2_ADDRESS)
                return_value = self.read_transmission(self.I2C_SLAVE2_ADDRESS)
                print(return_value)
                if return_value == 5:
                    break

            reset_testbed.cable_reset_spool_in()
            reset_testbed.cone_reset_down()
            reset_testbed.cable_reset_spool_out(self.spool_out_time_limit)
            reset_testbed.turntable_reset_home()


def on_exit(self, sig, func=None):
        gpio.cleanup()
        gpio.output(self.turntable_motor_in1, gpio.LOW)
        gpio.output(self.turntable_motor_in2, gpio.LOW)
if __name__ == '__main__':

    test_num = input("""
0) full table reset
1) spool_out
2) spool_in
3) cone_up
4) cone_down
5) turntable_home
6) turntable_angle
7) object_swap

What do you want to test? (enter the number)
""")


    reset_testbed = Testbed()
    if test_num == 0:
        reset_testbed.testbed_reset()

    elif test_num == 1:

        reset_testbed.cable_reset_spool_out(4.5)

    elif test_num == 2:
        reset_testbed.cable_reset_spool_in()

    elif test_num == 3:
        reset_testbed.cone_reset_up()
        
    elif test_num == 4:
        reset_testbed.cone_reset_down()
    
    elif test_num == 5:
        reset_testbed.turntable_reset_home()
    
    elif test_num == 6:
        angle = input("\nwhat angle do you want to rotate by?  (Degrees)\n")
        reset_testbed.turntable_move_angle(angle)
    elif test_num == 7:
        reset_testbed.object_swap()
    else:
        print("\nNot implemented\n")