"""
csci3302_lab2 controller.
Wallis McGuire, Augustin Pedro Garcia-Huidobro, Sam Shoemaker
"""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor
# import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
GROUND_SENSOR_THRESHOLD = 600

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1256 #m/s
MAX_SPEED = 6.28 #radians

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice(
    'gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10):
    robot.step(SIM_TIMESTEP)

# Initialize variable for left and right speed
vL = 0
vR = 0

# Set constant for startTime
STARTING_TIME = robot.getTime()

# variables for loop closure
TIME_SEEING_LINE_THRESHOLD = 0.1
line_seen = False
last_time_seeing_line = 0
LINE_SEEN_COOLDOWN = 1


def seeing_start_line(gsr):
    return gsr[CENTER_IDX] < GROUND_SENSOR_THRESHOLD and gsr[RIGHT_IDX] < GROUND_SENSOR_THRESHOLD and gsr[LEFT_IDX] < GROUND_SENSOR_THRESHOLD

def update_odometry(vL, vR):
    global pose_x, pose_y, pose_theta
    linear_l = (vL / leftMotor.getMaxVelocity()) * EPUCK_MAX_WHEEL_SPEED
    linear_r = (vR / rightMotor.getMaxVelocity()) * EPUCK_MAX_WHEEL_SPEED
    v = ((linear_l / 2) + (linear_r / 2))
    # Effective wheel separation from a separate Webots turning calibration:
    # right wheel at half speed, left stopped: 10.579225 rad in 9.6 s.
    effective_axle_diameter = (0.5 * EPUCK_MAX_WHEEL_SPEED) * 9.6 / 10.579225
    omega = (linear_r - linear_l) / effective_axle_diameter
    xI_dot = math.cos(pose_theta) * v
    yI_dot = math.sin(pose_theta) * v
    theta_dot = omega
    delta_t = SIM_TIMESTEP/1000
    # integrate to get position
    pose_x     += xI_dot * delta_t
    pose_y     += yI_dot * delta_t
    pose_theta += theta_dot * delta_t
    # Compare heading to zero using the equivalent angle between -pi and pi.
    # So we can be sure we fall within the correct error.
    pose_theta = math.atan2(math.sin(pose_theta), math.cos(pose_theta))

state = "speed_measurement"

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    match state:
    # Part 1
        case "speed_measurement":
            state = "line_follower"
            # vL = MAX_SPEED
            # vR = MAX_SPEED
            # if seeing_start_line(gsr):
            #     vL = 0
            #     vR = 0
            #     print(robot.getTime() - STARTING_TIME)
                # state = "line_follower"

    # Part 2
        case "line_follower":
            update_odometry(vL, vR)
            # Use 65% of maximum speed forward and 20% for in-place turns.
            # Center Sensor detects line -> drive forward
            if gsr[CENTER_IDX] < GROUND_SENSOR_THRESHOLD:
                vL = 0.65 * leftMotor.getMaxVelocity()
                vR = 0.65 * rightMotor.getMaxVelocity()
            # Right Sensor detects line -> rotate clockwise in place
            elif gsr[RIGHT_IDX] < GROUND_SENSOR_THRESHOLD:
                vL = 0.2 * leftMotor.getMaxVelocity()
                vR = -0.2 * rightMotor.getMaxVelocity()
            # Left Sensor detects line -> rotate counter-clockwise in place
            elif gsr[LEFT_IDX] < GROUND_SENSOR_THRESHOLD:
                vL = -0.2 * leftMotor.getMaxVelocity()
                vR = 0.2 * rightMotor.getMaxVelocity()
            # No sensors detect line -> rotate counter-clockwise to reacquire
            else:
                vL = -0.2 * leftMotor.getMaxVelocity()
                vR = 0.2 * rightMotor.getMaxVelocity()


            if seeing_start_line(gsr):
                if line_seen == False:
                    last_time_seeing_line = robot.getTime()
                line_seen = True
                if robot.getTime() - last_time_seeing_line >= TIME_SEEING_LINE_THRESHOLD:
                    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
                    pose_x, pose_y, pose_theta = 0, 0, 0
                    line_seen = False
            else:
                line_seen = False

    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
