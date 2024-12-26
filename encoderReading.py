# import RPi.GPIO as gpio
# import time


encoder1_A = 12
encoder1_B = 13
encoder2_A = 15
encoder2_B = 16

gpio.setmode(gpio.BOARD)
# encoder 1 setup
gpio.setup(encoder1_A, gpio.IN)
gpio.setup(encoder1_B, gpio.IN)
# encoder 2 setup
gpio.setup(encoder1_A, gpio.IN)
gpio.setup(encoder1_B, gpio.IN)



# the loop of the code
while True:
