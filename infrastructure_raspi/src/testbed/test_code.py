from time import time
#import spidev
import RPi.GPIO as gpio



if __name__ == '__main__':

    gpio.setup(button_pin, gpio.IN) #, pull_up_down = gpio.PUD_UP)
    
    while True:

        if gpio.input(button_pin) == HIGH:
            print("button was pressed!!!!!!!!!!!")
        