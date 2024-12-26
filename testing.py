# import numpy as np
from math import atan2, sqrt, cos, sin

# import time
# from scipy.integrate import quad

pi = 3.14159265359
L = 27.5
r = 3.2
x_0 = 0
x_actual = 0
y_0 = 0
y_actual = 0
theta_0 = 0
theta_actual = 0
x_desired = 10
y_desired = 10
x_final = x_desired
y_final = y_desired
counter = 0
prev_time = 0
dt = 0.2
# w = 0
# for i in range(0, 300):
#     current_time = time.time()
#     startcount = counter
#     time.sleep(0.002)
#     delta_time = current_time - prev_time
#     stopcount = counter
#     prev_time = current_time


# R = 100
# w = (theta_actual - theta_0)/delta_time
print(f"DT= {dt}")
should_go = True
while should_go:
    # for z in range (0, 10):
    x_err = x_desired - x_actual
    y_err = y_desired - y_actual
    theta_desired = atan2(y_err, x_err) * (180 / pi)
    theta_diff = theta_desired - theta_actual
    theta_err = atan2(sin(theta_diff), cos(theta_diff)) * (180 / pi)
    v = sqrt(x_err ** 2 + y_err ** 2)
    w = 2 * theta_err
    vr = (2*v + w * L) / (2)
    vl = (2*v - w * L) / (2)

    print(f"V_before= {v} , W_before= {w}")

    x_dot = ((vr + vl) * cos(theta_err)) / 2
    y_dot = ((vr + vl) * sin(theta_err)) / 2
    w = (vr - vl) / L
    # vr = v + (L/2) * w
    # vl = v - (L/2) * w

    x_actual = x_0 + v * cos(theta_actual - theta_0) * dt
    y_actual = y_0 + v * sin(theta_actual - theta_0) * dt
    theta_actual = theta_0 + w * (180 / pi) * dt
    # theta_actual = theta_0 + atan2(y_desired-y_actual, x_desired-x_actual) * (180/pi) * delta_time

    x_0 = x_actual
    y_0 = y_actual
    theta_0 = theta_actual
    counter += 1

    print(f"X_err= {x_err} , Y_err= {y_err}")
    print(f"Desired Theta= {theta_desired}")
    print(f"Desired Diff= {theta_diff}")
    print(f"Theta_err= {theta_err}")
    print(f"V_after= {v} , W_after= {w}")
    # print(f"WL= {wl} , WR= {wr}")
    print(f"VL= {vl} , VR= {vr}")
    print(f"X_actual= {x_actual} , Y_actual= {y_actual}")
    print(f"Theta_actual= {theta_actual}")

    print(f"Counts= {counter}\n\n")
    # print(f"time= {dt}")

    if x_err <= 0.0001*x_final and y_err <= 0.0001*y_final:
        should_go = False