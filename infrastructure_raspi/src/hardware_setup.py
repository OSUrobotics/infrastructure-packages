#!/usr/bin/env python

from time import time
import spidev
import RPi.GPIO as gpio
import VL53L0X
from stepper_motor import StepperMotor

#from drawer.drawer import Drawer

#global variables used across all hardware nodes
#def init():
#    global drawer 
#    drawer = Drawer()


tof = VL53L0X.VL53L0X()
spi_lower = spidev.SpiDev()
#spi_lower.open(1, 0) #seperate bus, can be used for multiprocessing
spi_lower.open(0, 1) #same bus, synchronous
spi_lower.max_speed_hz = 1000000
spi_upper = spidev.SpiDev()
spi_upper.open(0, 0)#bus for mcp3008 in charge of FSR's 8 - 12
spi_upper.max_speed_hz = 1000000

reset_pul = 5 #pin 29
reset_dir = 6 # pin 31 
reset_en = 13  # pin 33, (High to Enable / LOW to Disable)
time_unwind = 2 #in seconds
reset_speed = .000001 # seconds
dis_buffer = 5 #buffer value for resetting drawer (in mm)
fric_pul = 17 #pin 11
fric_dir = 27 # pin 13
fric_en = 22  # pin 15 (High to Enable / LOW to Disable)
fric_steps = .00032 #relation between friction to # of motor steps
fric_speed = .000001 #seconds
fric_min_steps = 2500 #min steps it takes to get brake to touch drawer
base_friction = 0.3 #minimum resistance drawer has (in kg)

gpio.setmode(gpio.BCM) #sets pin mapping to GPIO pins
gpio.setup(reset_pul, gpio.OUT)
gpio.setup(reset_dir, gpio.OUT)
gpio.setup(reset_en, gpio.OUT)
gpio.setup(fric_pul, gpio.OUT)
gpio.setup(fric_dir, gpio.OUT)
gpio.setup(fric_en, gpio.OUT)

reset_motor = StepperMotor(reset_pul, reset_dir, reset_en)
fric_motor = StepperMotor(fric_pul, fric_dir, fric_en)

 # def __set_friction(:
 #   fric_motor.step(__resistance_steps, fric_motor.CW)
 #   gpio.output(fric_en, gpio.HIGH) #keep motor resistance on
 # 
 # def __reset_friction(:
 #   fric_motor.step_to(0)
 #   
 # def __read_handle(:
 #   data = [-1] * 14
 #   #lower ADC
 #   for chan in range(0,8):
 #     r = spi_lower.xfer2([1, 8 + chan << 4, 0])
 #     data[chan] = int(((r[1] & 3) << 8) + r[2])
 #   #spidev automatically switches CS signals
 #   #upper ADC
 #   for chan in range(0,4):
 #     r = spi_upper.xfer2([1, 8 + chan << 4, 0])
 #     data[chan + 8] = int(((r[1] & 3) << 8) + r[2])
 #   return data
 #   
 # 
 # def start_new_trial( resistance, tof_mode = 0):
 #   tof_mode = tof_mode
 #   __resistance_steps = int(((resistance - base_friction) / fric_steps) + fric_min_steps)
 #   tof.start_ranging(tof_mode)
 #   start_pos = tof.get_distance()
 #   __set_friction()
 #   #__clean_trial() 
 #   __trial_data = [] #not need?
 # 
 # def collect_data(:
 #   #data_point = DataPoint(tof.get_distance() - start_pos, __read_handle(), time())
 #   #__trial_data.append(data_point)
 #   data_point = DataPoint(tof.get_distance() - start_pos, __read_handle())
 #   return data_point
 # 
 # def reset(:
 #   __reset_friction()
 #   did_move = False
 #   end_pos = start_pos + dis_buffer
 #   while (True):
 #     if(tof.get_distance() <= end_pos):
 #       break
 #     did_move = True
 #     reset_motor.move_for(0.1, reset_motor.CCW)
 #   if(did_move):
 #     reset_motor.move_for(time_unwind, reset_motor.CW)
 #   tof.stop_ranging() 
