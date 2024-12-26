import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(33, GPIO.OUT)
my_pwm = GPIO.PWM(33, 100)
my_pwm.start(5)
my_pwm.ChangeDutyCycle(75)
