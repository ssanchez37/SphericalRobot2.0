"""test_controllerpy controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot

TIME_STEP = 64

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
longitudinal = robot.getDevice('long_motor')
longitudinal.setPosition(0.0)
longitudinal.setVelocity(0.0)
steer = robot.getDevice('steer_motor')
steer.setPosition(0.0)
steer.setVelocity(0.0)
my_gps = robot.getDevice('my_gps')
my_gps.enable(timestep)
long_pos = robot.getDevice('long_pos')
long_pos.enable(timestep)
inertial = robot.getDevice('inertial')
inertial.enable(timestep)
inertialrod = robot.getDevice('inertialrod')
inertialrod.enable(timestep)
gyro1 = robot.getDevice('gyro1')
gyro1.enable(timestep)
gyroshell = robot.getDevice('gyroshell')
gyroshell.enable(timestep)
theta2 = robot.getDevice('steer_pos')
theta2.enable(timestep)

# Initialization of parameters
error = 0
previous_error = 0
error_integral = 0
error_derivative = 0
ts = timestep / 1000
j = 1
k = 0
pi = math.pi
xa = 0
ya = 0
za = 0.198
x1a = 0
y1a = 0
z1a = 0.198
sp = 0
spz = 5
spz2 = -5
r1 = 0.2
r2 = 0.18
m1 = 0.5
m2 = 0.639
ic = 0.05
g = 9.81
rG = m2 * r2 / (m1 + m2)
Vd = -4

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    pos = my_gps.getValues()
    theta = inertial.getRollPitchYaw()
    theta2 = inertialrod.getRollPitchYaw()
    thetap = gyro1.getValues()
    phip = gyroshell.getValues()
    beta = long_pos.getValue()

    # Process sensor data here.
    x = pos[0]
    y = pos[2]
    z = pos[1]
    th1 = theta[1]
    thp1 = thetap[2]
    th2 = pi + theta[0]
    phi = beta - th1
    thetadeg = theta[1] * 180 / pi
    betadeg = beta * 180 / pi
    phideg = phi * 180 / pi
    th2deg = th2 * 180 / pi

    x1 = x
    y1 = y
    z1 = 0.198
    maxTorque = 0.68
    minTorque = -0.68

    px = x - xa
    py = y - ya
    pz = z - za
    px1 = x1 - x1a
    py1 = y1 - y1a
    pz1 = z1 - z1a
    ppunto = (px * px1) + (py * py1) + (pz * pz1)
    psqr = (px ** 2) + (py ** 2) + (pz ** 2)
    p1sqr = (px1 ** 2) + (py1 ** 2) + (pz1 ** 2)
    pmod = (psqr ** 0.5) * (p1sqr ** 0.5)
    parg = ppunto / pmod

    if k < 5:
        alpha = 0
    else:
        alpha = math.acos(parg)

    if (pz >= 0) or ((pz < 0) and (phip[1] > 0)):
        alphadeg = alpha * 180 / pi
    else:  # ((pz < 0)and(phip[1] < 0)):
        alphadeg = -alpha * 180 / pi

    alphadeg = int(alphadeg)

    # if alphadeg < 0:
    #     alphadeg = int(alphadeg) - 1
    # else:
    #     alphadeg = int(alphadeg) + 1

    # PID controller tuned using stochastic signals
    kp = -43.7707018  # Proportional Gain
    ki = -2.9050182  # Integral Gain
    kd = -0.2250206  # Derivative Gain

    error = Vd - phip[1]
    error_integral = (error_integral + error) * ts
    error_derivative = (error - previous_error) / ts

    if (alphadeg < 1) and (alphadeg >= -1) and (phip[1] < 1):
        newTorque = kp * error + ki * error_integral + kd * error_derivative
        maxTorque = 0.1
        minTorque = -maxTorque
    else:
        newTorque = kp*error + ki*error_integral + kd*error_derivative
        maxTorque = alphadeg * 0.0488
        minTorque = -maxTorque
        # newTorque = alphadeg * 0.06

    if newTorque > maxTorque:
        newTorque = maxTorque
    elif newTorque < minTorque:
        newTorque = minTorque
    else:
        newTorque = newTorque

    previous_error = error
    xa = x
    ya = y
    za = z
    x1a = x1
    y1a = y1
    z1a = z1

    tiempo = j * 0.032

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    if tiempo < 47:
        if x < sp:
            longitudinal.setTorque(newTorque)
            rc = (spz2 - spz) / 2
            thsteer = (((m1 + m2) * r1 + m2 * (r1 - r2)) * ((phip[1] ** 2) * (r1 ** 2)) + m2 * g * r2 * r1) / (
                        m2 * g * r2 * rc)
            thsteer = (1 - 0.01) * (thsteer - 0.032 * thetap[2]) + 0.01 * th2
            steer.setPosition(thsteer)
            steer.setVelocity(1.5)
            # steer.setControlPID(1.7, 1.74, 0.01)
            # steer.setControlPID(0.13, 0.00001, 0.00001)
            steer.setControlPID(0.1, 0.0015, 0.001)
            aqui = 1
            testeer = thsteer * 180 / pi
        elif (x >= sp) and (y >= 0):
            longitudinal.setTorque(newTorque)
            rc = (0 - spz) / 2
            thsteer = (((m1 + m2) * r1 + m2 * (r1 - r2)) * ((phip[1] ** 2) * (r1 ** 2)) + m2 * g * r2 * r1) / (
                        m2 * g * r2 * rc)
            thsteer = (1 - 0.01) * (thsteer - 0.032 * thetap[2]) + 0.01 * th2
            steer.setPosition(thsteer)
            steer.setVelocity(1.5)
            # Tune PID Control Variables
            # steer.setControlPID(1.7, 1.74, 0.01)
            steer.setControlPID(0.68, 0.2, 0.005)
            aqui = 2
            testeer = thsteer * 180 / pi
        elif (x >= sp) and (y < 0):
            longitudinal.setTorque(newTorque)
            rc = 0 - spz / 2
            thsteer = (((m1 + m2) * r1 + m2 * (r1 - r2)) * ((phip[1] ** 2) * (r1 ** 2)) + m2 * g * r2 * r1) / (
                        m2 * g * r2 * rc)
            thsteer = (1 - 0.01) * (thsteer - 0.032 * thetap[2]) + 0.01 * th2
            steer.setPosition(thsteer)
            steer.setVelocity(1.5)
            # steer.setControlPID(1.7, 1.74, 0.01)
            # Best value for kp = 0.3, ki = 0.05, kd = 0.01
            steer.setControlPID(0.3, 0.05, 0.01)
            aqui = 3
            testeer = thsteer * 180 / pi
    elif tiempo >= 47:
        longitudinal.setTorque(newTorque)
        steer.setPosition(0)
        steer.setVelocity(1.5)
        steer.setControlPID(1.7, 1.74, 0.01)
        aqui = 4

    j += 1

    print('Theta1 = ', thetadeg, '°')
    print('Theta2 = ', th2deg, '°')
    print('Beta = ', betadeg, '°')
    print('Phi = ', phideg, '°')
    print('X', x, 'm')
    print('Y', y, 'm')
    print('Z', z, 'm')
    print('X1', px1, 'm')
    print('Y1', py1, 'm')
    print('Z1', pz1, 'm')
    print('W', thp1, 'rad/s')
    print('Wsph', phip[1], 'rad/s')
    print('Debug1', newTorque)
    # noinspection PyUnboundLocalVariable
    print('Debug2', aqui)
    print('Debug3', alphadeg)
    k += 1

    pass

# Enter here exit cleanup code.
