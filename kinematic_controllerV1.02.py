import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as mat
from math import atan2, sqrt, cos, sin, pi
from scipy.integrate import quad
import numpy as np


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]


def getpose():

    pass


def encoder_handler(A):
    global counter
    global prev_counter
    global delta_ticks
    prev_counter = counter
    counter += 1
    delta_ticks = counter - prev_counter


def setMotor(dire, pwmVal, in1, in2, motor):
    if dire == 1:
        GPIO.output(in1, 1)
        GPIO.output(in2, 0)
    elif dire == -1:
        GPIO.output(in1, 0)
        GPIO.output(in2, 1)
    else:
        GPIO.output(in1, 0)
        GPIO.output(in2, 0)
    my_pwm2.start(pwmVal)
    if motor == 2:
        my_pwm2.start(pwmVal)
    elif motor == 1:
        my_pwm1.start(pwmVal)


def setting_up_GPIO():
    GPIO.cleanup()
    m1_enA = 11
    m1_enB = 12
    m2_enA = 15
    m2_enB = 13
    global m2_input1
    m2_input1 = 23
    global m2_input2
    m2_input2 = 19
    global m1_input1
    m1_input1 = 22
    global m1_input2
    m1_input2 = 21
    pwm2 = 33
    pwm1 = 32

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(m1_enA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(m2_enA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pwm2, GPIO.OUT)
    GPIO.setup(m2_input1, GPIO.OUT)
    GPIO.setup(m2_input2, GPIO.OUT)
    GPIO.setup(pwm1, GPIO.OUT)
    GPIO.setup(m1_input1, GPIO.OUT)
    GPIO.setup(m1_input2, GPIO.OUT)
    global my_pwm2
    my_pwm2 = GPIO.PWM(pwm2, 100)
    global my_pwm1
    my_pwm1 = GPIO.PWM(pwm1, 100)

    GPIO.add_event_detect(m1_enA, GPIO.RISING, encoder_handler)
    GPIO.add_event_detect(m2_enA, GPIO.RISING, encoder_handler)


def pid_motor(pid_x, pid_y, pid_x0, pid_y0, pid_theta, pid_startcounter, pid_stopcounter, pid_delta_time,
              pid_w, pid_v, pid_kp, pid_ki, pid_kd,
              pid_e_integral, pid_dedt, pid_e, pid_e_prev):
    delta_ticks_left = 300 * delta_ticks
    delta_ticks_right = 300 * delta_ticks
    Dc = (300*delta_ticks)
    x_dot = Dc * cos((pid_theta))
    x_error = pid_x - pid_x0
    y_error = pid_y - pid_y0
    theta_goal = atan2(y_error, x_error) * (180 / pi)
    pid_e = theta_goal - pid_theta
    pid_e = atan2(sin(pid_e), cos(pid_e))
    # pid_w = pid_v / r
    # pid_speed_actual = round((pid_stopcounter - pid_startcounter) *360/(pid_delta_time*100/3)/300, 2)
    pid_dedt = (pid_e - pid_e_prev) / pid_delta_time
    pid_e_integral = pid_e_integral + pid_e
    pid_u = pid_e * pid_kp + pid_dedt * pid_kd + pid_e_integral * pid_ki
    pid_e_prev = pid_e
    # here there is a problem in the previous error
    return pid_u, x_error, y_error, theta_goal


def speed_estimator(v, w):
    vr = (v + 0.23 * w / 2) * 0.032
    vl = (v - 0.23 * w / 2) * 0.032
    return vl, vr


def stop():
    my_pwm1.ChangeDutyCycle(0)
    my_pwm2.ChangeDutyCycle(0)


# constants
l = 0.23  # meter
r = 0.032  # radius meter
v = 0.03  # car's speed ##################

#  global variables:
# position variables
# encoder variables:
counter = 0
prev_counter = 0

t = []
counting = []
# time and counter variables
startcount, stopcount, prev_time, time0 = 0, 0, 0, 0
# plotting variables
speed_output1 = []
speed_output2 = []
wr_output = []
wl_output = []
time_output = []
# working of the motor's direction
motor = 0
dire1 = 0
dire2 = 0
# PID variables
e_prev_1, e_prev_2, e_integral_1, e_integral_2 = 0.0, 0.0, 0.0, 0.0
dedt_1, dedt_2, e1L, e2R, u1, u2 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
e_prev, e, u, dedt = 0.0, 0.0, 0.0, 0.0
e_integral = 0.0
pwr1L, pwr2R = 0, 0
# PID constants
Kp_1, Kp_2 = 0.2, 0.2
Kd_1, Kd_2 = 0.003, 0.003
Ki_1, Ki_2 = 0.0, 0.0

# 1d-table value:
motor_1_speed_list = [-79.82, -77.85, -77.73, -77.13, -77.05, -76.78, -76.55, -75.93, -75.75, -75.54, -75.12, -75.1,
                      -74.86, -74.7, -74.64, -74.33, -74.21, -74.15, -74.06, -73.79, -73.32, -72.9, -72.08, -71.4,
                      -71.21, -70.97, -70.72, -70.37, -70.13, -69.82, -68.74, -68.63, -68.6, -68.58, -68.03, -67.83,
                      -67.79, -67.37, -67.03, -65.26, -64.67, -64.5, -63.81, -63.07, -62.93, -62.74, -62.73, -62.16,
                      -61.03, -60.72, -57.95, -57.4, -56.93, -55.83, -54.84, -53.44, -52.02, -51.3, -51.29, -49.09,
                      -48.27, -44.8, -44.53, -41.66, -40.87, -39.14, -37.93, -35.91, -34.9, -33.15, -31.05, -29.8,
                      -27.5, -23.6, -21.53, -20.64, -18.85, -18.12, -16.16, -13.34, -12.53, -10.27, -7.43, -7.11, -6.85,
                      -4.53, -4.5, -3.35, -2.29, -2.28, -1.15, -1.15, -1.14, -1.13, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
                      -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.15, 0.0, 0.0, 0.0, 0.0, 1.14, 1.14, 3.45, 4.59,
                      5.72, 8.05, 9.12, 11.35, 12.54, 14.54, 15.98, 19.59, 21.59, 22.8, 25.38, 27.39, 29.84, 30.09,
                      32.76, 35.14, 35.96, 38.58, 40.03, 40.7, 43.99, 44.94, 45.81, 47.52, 49.21, 51.17, 51.99, 55.73,
                      51.51, 55.75, 57.98, 57.19, 58.37, 59.8, 61.19, 60.73, 61.34, 65.94, 60.7, 63.66, 64.09, 64.61,
                      60.83, 65.24, 66.75, 67.81, 69.33, 67.83, 71.37, 67.88, 70.67, 69.84, 69.93, 66.6, 73.58, 68.8,
                      74.5, 66.62, 71.73, 72.03, 71.19, 74.31, 74.07, 73.38, 75.07, 75.13, 74.33, 74.2, 73.82, 74.52,
                      77.55, 72.21, 75.84, 73.3, 72.86, 76.19, 77.82, 80.21, 71.56, 77.21, 78.54, 77.12, 78.29, ]
motor_1_pwm = []
for i in range(-100, 100):
    motor_1_pwm.append(i)

motor_2_speed_list = [-85.6, -83.35, -81.36, -80.32, -80.23, -80.19, -80.0, -79.77, -79.75, -79.71, -78.6, -78.54,
                      -78.51, -78.26, -77.83, -77.73, -76.65, -76.53, -76.01, -75.88, -75.2, -75.16, -75.11, -75.07,
                      -74.87, -74.41, -74.35, -74.17, -74.1, -73.91, -73.63, -73.44, -72.55, -72.08, -71.76, -71.5,
                      -71.41, -71.11, -69.34, -68.93, -68.74, -68.11, -66.33, -65.56, -65.13, -64.62, -62.82, -61.21,
                      -59.99, -58.87, -58.7, -58.41, -55.02, -54.39, -54.09, -53.66, -53.04, -52.93, -52.27, -50.58,
                      -49.4, -47.38, -45.64, -44.07, -43.36, -40.64, -39.73, -37.96, -35.61, -34.65, -31.32, -28.9,
                      -26.59, -25.84, -23.61, -21.8, -19.22, -18.28, -14.81, -12.75, -11.57, -9.24, -8.04, -5.7, -4.62,
                      -2.29, -2.29, -1.15, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
                      0.0, 0.0, 0.0, 0.0, 1.15, 1.15, 0.0, 0.0, 0.0, 0.0, 1.14, 0.0, 1.15, 1.13, 3.42, 3.37, 5.7, 6.68,
                      9.19, 10.32, 11.49, 14.65, 15.93, 18.15, 19.13, 22.78, 23.95, 20.56, 28.7, 29.39, 34.32, 34.53,
                      36.37, 38.98, 39.94, 42.94, 45.82, 44.69, 46.43, 48.69, 48.49, 46.16, 52.51, 54.19, 51.07, 54.54,
                      57.84, 56.89, 53.77, 60.56, 58.67, 64.17, 61.11, 57.04, 67.75, 60.29, 59.17, 63.09, 63.69, 61.95,
                      68.14, 58.64, 62.75, 65.09, 71.94, 68.27, 73.5, 72.35, 72.63, 70.61, 65.82, 72.08, 70.11, 73.19,
                      75.3, 75.02, 74.41, 74.71, 72.52, 70.43, 69.92, 72.14, 66.6, 65.66, 73.96, 65.88, 75.93, 74.39,
                      69.73, 68.2, 70.63, 71.39, 73.57, 68.6, 72.8, 74.11, 73.62, 78.22, 76.27, 81.57, ]
motor_2_pwm = []
for i in range(-100, 100):
    motor_2_pwm.append(i)

# start of the code:

setting_up_GPIO()

# wl = (1 / r) * (v - l * (v / r) / 2)
# wr = (1 / r) * (v + l * (v / r) / 2)

nearest_val1 = find_nearest(motor_1_speed_list, wl)
print(f"The nearest value in Speed1 list: {nearest_val1}")
position_in_list = motor_1_speed_list.index(nearest_val1)
print(f"Position of n in speed1 list: {position_in_list}")
wl = nearest_val1

nearest_val2 = find_nearest(motor_2_speed_list, wr)
print(f"The nearest value in Speed2 list: {nearest_val2}")
position_in_list = motor_2_speed_list.index(nearest_val2)
print(f"Position of n in speed2 list: {position_in_list}")
wr = nearest_val2

# for loop
for i in range(0, 300):
    current_time = time.time()
    startcount = counter
    time.sleep(0.002)
    delta_time = current_time - prev_time
    stopcount = counter
    prev_time = current_time
    # don't forget tp add the wl and wr from the list above

    speed_actual1 = round((stopcount - startcount) * 360 / (delta_time * 100 / 3) / 300, 2)
    speed_actual2 = round((stopcount - startcount) * 360 / (delta_time * 100 / 3) / 300, 2)

    pid_thing, x1, y1, theta1 = pid_motor(2, 2, 0, 0, 0, startcount, stopcount, delta_time, 3, 4, 0.2, 0.0, 0.003, 0, 0,
                                          0, 0)
    wr, wl = speed_estimator(v, pid_thing)
    pwr1L = abs(int(motor_1_speed_list.index(find_nearest(motor_1_speed_list, wl))))
    pwr2R = abs(int(motor_2_speed_list.index(find_nearest(motor_2_speed_list, wr))))
    if pwr1L > 100:
        pwr1L = 100
    if wl < 0:
        dire1 = -1
    else:
        dire1 = 1

    if pwr2R > 100:
        pwr2R = 100
    if wr < 0:
        dire2 = -1
    else:
        dire2 = 1
    print(pwr1L)
    print(pwr2R)

    setMotor(dire1, pwr1L, m1_input1, m1_input2, 1)
    setMotor(dire2, pwr2R, m2_input1, m2_input2, 2)
    speed_output1.append(speed_actual1)
    speed_output2.append(speed_actual2)
    wr_output.append(pwr2R)
    wl_output.append(pwr1L)

    time_output.append(time0)

    print(f"Wl: {wl}\nXerror: {x1}\nTerror: {y1}\ntheta error: {theta1} ")

    print("#################################################################")

print("12314314124")
# i = 0
# while i < 10:


print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSs")
stop()
mat.title("PD controller - M1 ")
mat.ylabel("speed / deg/s")
mat.xlabel("time")
mat.plot(time_output, speed_output2, label="m2")
mat.plot(time_output, speed_output1, label="m1")
mat.plot(time_output, wr_output, label="wr")
mat.plot(time_output, wl_output, label="wl")
mat.legend()
mat.grid()
mat.show()


