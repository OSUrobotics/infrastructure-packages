from time import time, sleep
import spidev 
import RPi.GPIO as gpio
import sys
import smbus2 as smbus#,smbus2

class testbed_test():
    def __init__(self):
        
        # Slave Addresses
        self.I2C_SLAVE_ADDRESS = 12 #0x0b ou 11
        self.I2C_SLAVE2_ADDRESS = 13
        # self.button_pin = 24
        self.button_pin = 14

        self.motor_pwm = 4 # pin 7
        self.motor_in1 = 21 # pin 
        self.motor_in2 = 27 # pin 
        self.motor_en = 13

        # self.hall_effect_pin = 
        self.hall_effect_pin = 23
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
        self.p.start(75)
        
        gpio.setup(self.hall_effect_pin, gpio.IN, pull_up_down= gpio.PUD_DOWN)
        
        self.spi_bus = 0
        self.spi_device = 0
        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, self.spi_device)
        self.spi.max_speed_hz = 1000000

    def ConvertStringsToBytes(self, src):
        self.converted = []
        try:  
            for b in src:
                self.converted.append(ord(b))
            return self.converted
        except:
            self.converted.append(ord(src))
            return self.converted

    def button_test(self):
        i = 0
        while True:
            
            if gpio.input(self.button_pin) == gpio.HIGH:
                print("{} button was pressed!!!!!!!!!!!".format(i))
                i += 1
            # sleep(.01)

    def motor(self, duration=None):
        gpio.output(self.motor_in1, gpio.LOW)
        gpio.output(self.motor_in2, gpio.HIGH)
        if duration == None:

            while True:
                
                # gpio.output(self.motor_in1, gpio.LOW)
                # gpio.output(self.motor_in2, gpio.HIGH)

                sleep(1)
        else:
            gpio.output(self.motor_in1, gpio.LOW)
            gpio.output(self.motor_in2, gpio.HIGH)
            sleep(duration)

            gpio.output(self.motor_in1, gpio.LOW)
            gpio.output(self.motor_in2, gpio.LOW)
            gpio.cleanup()
    
    def motor_with_encoder(self):
        pass

    def talk_with_arduino(self, angle=10):

        start_counting = 1
        encoder_val = 2
        get_val = 3
        # get_val_p2 = 4
        stop_counting = 4

        self.spi.xfer2([start_counting])
        sleep(0.0001)

        while True:

            # encoder_val = [2]
            # get_val_p1 = [3]
            # get_val_p2 = [4]
            # stop_counting = [5]

            self.spi.xfer2([encoder_val])
            sleep(0.0001)
            encoder_value_part1 = self.spi.xfer2([get_val])
            sleep(0.0001)
            encoder_value_part2 = self.spi.xfer2([0])
            sleep(0.0001)

            value_part1 = encoder_value_part1[0] << 8
            encoder_value = value_part1 + encoder_value_part2[0]

            print(encoder_value)

            if encoder_value >= angle:
                
                break
        
        self.spi.xfer2([stop_counting])

    def hall_effect(self):
        counter = 0

        while True:
            if gpio.input(self.hall_effect_pin) == False:
                print("{}  magnet is detected".format(counter))
                counter += 1
            # print(gpio.input(self.hall_effect_pin))

            sleep(.01)
    
    def hall_effect_arduino(self):
        self.I2Cbus = smbus.SMBus(1)
        with smbus.SMBus(1) as I2Cbus:
            sleep(0.001)
            # self.BytesToSend = self.ConvertStringsToBytes("5")
            while True:
                try:
                    sleep(0.1)
                    self.I2Cbus.write_i2c_block_data(self.I2C_SLAVE_ADDRESS, 0x00, [5])
                    sleep(0.1)
                    break
                except:
                    sleep(1)
                    self.I2Cbus.write_i2c_block_data(self.I2C_SLAVE_ADDRESS, 0x00, [5])
                    sleep(1)

            while True:
                try:
                    data=self.I2Cbus.read_byte_data(self.I2C_SLAVE_ADDRESS,1)
                    print("recieve from slave:")
                    print(data)
                    sleep(.01)
                except:
                    print("remote i/o error")
                    sleep(2)
        return 0

    def button_arduino(self):
        counter = 0

        self.spi.xfer2([6])
        while True:
            button_val = self.spi.xfer2([6])

            print("{} button value: {}".format(counter, button_val[0]))

    def limit_switch_aduino(self):
        counter = 0

        self.spi.xfer2([7])
        while True:
            limit_switch_val = self.spi.xfer2([7])

            print("{} limit switch value: {}".format(counter, limit_switch_val[0]))
            counter += 1

if __name__ == '__main__':
    gpio.cleanup()
        

    test_num = input("""
    1) button
    2) motor
    3) motor and encoder
    4) stepper motor
    5) Arduino Communication
    6) hall effect sensor
    7) hall effect arduino
    8) button with arduino
    9) limit switch arduino

    What do you want to test? (enter the number)
    """)


    test = testbed_test()

    if test_num == 1:

        test.button_test()

    elif test_num == 5:
        test.talk_with_arduino()
    elif test_num == 6:
        test.hall_effect()
    
    elif test_num == 7:
        test.hall_effect_arduino()

    elif test_num == 8:
        test.button_arduino()
    
    elif test_num == 9:
        test.limit_switch_aduino()
        
    elif test_num == 2:
        
        # try:
        #     duration = input("how long to run motors? \n(Enter None if you want to run indefently)\n")
        # except SyntaxError:
        # if len(duration) == None:
            # print("true")

        # test.motor(duration=duration)
        test.motor()
    else:
        print("not implemented yet")
