import time
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
from dronekit import connect, VehicleMode
from aruco_detection import arm_and_takeoff
from pymavlink import mavutil
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
# print("Get some vehicle attribute values:")
# print( " GPS: %s" % vehicle.gps_0)
# print (" Battery: %s" % vehicle.battery)
# print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
# print (" Is Armable?: %s" % vehicle.is_armable)
# print (" System status: %s" % vehicle.system_status.state)
# print (" Mode: %s" % vehicle.mode.name )   # settable


def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


arm_and_takeoff(15)

# print( " GPS: %s" % vehicle.gps_0)
# bay_ngang(150, 20)
# print( " GPS: %s" % vehicle.gps_0)


vehicle.mode = VehicleMode("LAND")


time.sleep(1)
while vehicle.armed:
	print("Waiting for drone to land...")
	time.sleep(1)
print("Drone has landed.")