import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
# from aruco_detection import arm_and_takeoff
from pymavlink import mavutil
from dronekit import connect
import math

MAX_PITCH = 40
THRESHOLD_X = 0.05  # khoảng cách đến landing pad nhỏ nhất để drone hạ cánh theo phương thẳng đứng, tránh trường hợp pitch ra vô cực
THRESHOLD_YAW = 30  # tốc độ xoay tối đa để cho phép drone bay theo phương ngang
MAX_SPEED_X = 10  #  m/s (Air 2S)
MIN_SPEED_X = 0.1  #  m/s TODO: need to test
MAX_DISTANCE_X = 100  # meters
MIN_DISTANCE_X = 1  # meter
MAX_SPEED_Z = 3  #  m/s (SDK)
MIN_SPEED_Z = 0.2  #  m/s TODO: need to test
MAX_DISTANCE_Z = 30  # meters
MIN_DISTANCE_Z = 2  # meter
# Connect to the Vehicle (in this case a UDP endpoint)

def arm_and_takeoff(vehicle, targetHeight):
    timeout = 0
    while vehicle.is_armable!=True and timeout < 5:
        print("Waiting for vehicle to become armable.")
        timeout = timeout +1
        time.sleep(1)
    print("Vehicle is now armable")
    vehicle.mode = VehicleMode("GUIDED")
    # while vehicle.mode!='GUIDED':
    #     print("Waiting for drone to enter GUIDED flight mode")
    #     time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
    vehicle.armed = True
    print("Taking off!")
    vehicle.simple_takeoff(targetHeight)  # Take off to target altitude

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)
def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]



def xoay(yaw_angle = 0.0, yaw_rate = 0.0):
    pre = 1 if yaw_angle >= 0 else -1
    if abs(yaw_angle) > yaw_rate:
        set_attitude(yaw_rate= pre * yaw_rate, duration=1.0)
    else:
        duration = round(abs(yaw_angle)/yaw_rate, 2)
        set_attitude(yaw_rate= pre * yaw_rate, duration= duration)

#bay ngang
def get_param_while_fly_horizontally(
    x,
    max_speed_x=MAX_SPEED_X,
    max_distance_x=MAX_DISTANCE_X,
    min_speed_x=MIN_SPEED_X,
    min_distance_x=MIN_DISTANCE_X,
):
    distance_x = abs(x)
    if distance_x > max_distance_x:
        drone_speed_x = max_speed_x
    elif distance_x < min_distance_x:
        drone_speed_x = min_speed_x
    else:
        drone_speed_x = max_speed_x * (distance_x / max_distance_x)
    pitch = np.sign(x) * drone_speed_x
    print(np.sign(x))
    return pitch
def sign(x):
    if x > 0:
        return 1
    if x < 0: 
        return -1
    if x == 0:
        return 0
def goto_position_target_local_ned(vehicle, x, y, z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111111000,
        x, y, z,
        0.5, 0.5, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    
    vehicle.flush()
# def bay_ngang(x):
#     vx = get_param_while_fly_horizontally(x)
#     send_local_ned_velocity(x, vx, 0, 0)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
​
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# def goto_position_target_local_ned(vehicle,north, east, down):
#     """	
#     Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
#     location in the North, East, Down frame.
# ​
#     It is important to remember that in this frame, positive altitudes are entered as negative 
#     "Down" values. So if down is "10", this will be 10 metres below the home altitude.
# ​
#     Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
#     ignored. For more information see: 
#     http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
# ​
#     See the above link for information on the type_mask (0=enable, 1=ignore). 
#     At time of writing, acceleration and yaw bits are ignored.
# ​
#     """
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#         0,       # time_boot_ms (not used)
#         0, 0,    # target system, target component
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
#         0b0000111111111000, # type_mask (only positions enabled)
#         north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
#         1.2,2.0, 0, # x, y, z velocity in m/s  (not used)
#         0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
#         0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
#     # send command to vehicle
#     print('lol')
#     vehicle.send_mavlink(msg)


# def arm_and_takeoff(vehicle,aTargetAltitude):
#     """
#     Arms vehicle and fly to aTargetAltitude.
#     """
#     print("Basic pre-arm checks")
#     # Don't try to arm until autopilot is ready
#     while  vehicle.is_armable:
#         print(" Waiting for vehicle to initialise...")
#         time.sleep(1)
#     print("Arming motors")
#     # Copter should arm in GUIDED mode
#     vehicle.mode = VehicleMode("AUTO")
#     vehicle.armed = True
#     # Confirm vehicle armed before attempting to take off
#     while  not vehicle.armed:
#         print(" Waiting for arming...")
#         time.sleep(1)
#     print("Taking off!")
#     vehicle.simple_takeoff(aTargetAltitude)



if __name__ == "__main__":
    vehicle = connect('0.0.0.0:14550', wait_ready=True)
    print(vehicle.gps_0)
    arm_and_takeoff(vehicle,10)
    time.sleep(10)
    goto_position_target_local_ned(vehicle, 24,67,-25)

    time.sleep(50)
    vehicle.mode = VehicleMode("AUTO")
    # condition_yaw(70, True)
    # time.sleep(10)

    goto_position_target_local_ned(vehicle, -14,-26,-30)
    time.sleep(35)
    vehicle.mode = VehicleMode("LAND")
    time.sleep(2)
    while vehicle.armed:
        print("Waiting for drone to land...")
        time.sleep(1)
    print("Drone has landed.")
    vehicle.mode = VehicleMode("RTL")
    print(vehicle._heading)
    time.sleep(1)
    vehicle.reboot()
    vehicle.close()