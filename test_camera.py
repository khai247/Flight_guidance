import time
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode
from smart_landing.rosetta_camera import RosettaCamera
from aruco_detection import get_vector_movement, get_coordinate
from flight_guidance import arm_and_takeoff, goto_position_target_local_ned
cam = RosettaCamera(debug=True)
cam.connect()

windowName = "Video Test"
cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
vehicle = connect('0.0.0.0:14550', wait_ready=True)
arm_and_takeoff(vehicle, 1.5)
time.sleep(10)
vehicle.gimbal.rotate(-90, 0, 0)
goto_position_target_local_ned(vehicle, 4.8, 3.7, -0.5)
print("den noi r ")
time.sleep(30)
while True :
	img,ts,yaw = cam.getImage()
	# img = cv2.resize(img, (5472, 3648), interpolation=cv2.INTER_CUBIC)
			# goto_position_target_local_ned(vehicle, x,y, z)
	img_re=cv2.resize(img,(1280,720))
	cv2.imshow("is",img_re)
	cv2.waitKey(25)
	if get_coordinate(img) is not None:
		
		(x_, y_), h = get_coordinate(img)
	# 	img = cv2.circle(img, (x_, y_), 16, (0, 0, 255), 1)
		
		x, y, z = get_vector_movement(img)
		print(x, y, z)
		goto_position_target_local_ned(vehicle, -x, -y, 0)
		time.sleep(3)
		goto_position_target_local_ned(vehicle, 0, 0, 0)
		
		cv2.imwrite("ccc.jpg", img)
		if (abs(x) <= 0.1 and abs(y) <= 0.1 and abs(z) <= 0.6):
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







# while True:




# 	# time.sleep(3)
# 	# 	# 	
# 	# 	print(x, y, z)
	
# 	if cv2.waitKey(1) & 0xFF == ord("q"):
#             break

# cv2.destroyAllWindows()