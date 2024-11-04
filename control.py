import time

from PID import *

# Arduino COM port
COM_PORT = "/dev/ttyACM0"

# Using mac?
USING_MAC = False

# Target values
MOTOR_TARGET_ANGLE = 90  # degrees
PENDULUM_TARGET_ANGLE = 0  # degrees
MOTOR_TARGET_RPM = 0  # rpm (max 3500)

pid = PID()


# Main function to control. Must return the voltage (control signal) to apply to the motor.
I = 0
prev_error = 0
D_filtered = 0
D = 0
timer = 0
voltage = 0

pid.kp = 0.01
pid.ki = 0.03
pid.kd = 0.0005
def control_system(dt, motor_angle, pendulum_angle, rpm):

#0.01 0.03 0.0005
    global timer
    global MOTOR_TARGET_ANGLE
    global MOTOR_TARGET_RPM

    global I
    global prev_error
    global D_filtered
    global D
    global voltage


    timer += dt

    error = MOTOR_TARGET_RPM - rpm

    P = pid.kp * error
    I += pid.ki * error * dt
    if(I > pid.windup):
        I = pid.windup

    if I < -pid.windup:
        I = -pid.windup

    D = pid.kd * (error - prev_error) / dt
    D_filtered = D *0.5 + D_filtered * 0.5


    prev_error = error

    u = P + I + D_filtered
    return u



# This function is used to tune the PID controller with the GUI.
def setPidParams(_pid):
    pid.copy(_pid)  # Uncomment this line if you want to use the GUI to tune your PID live.
    return 0