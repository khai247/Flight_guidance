import random
import time
import cv2
from cv2 import aruco
import numpy as np
import math
import time
from math import radians
from dronekit import Vehicle
from statistics import mean
from coordinate_calculation import PAD

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket

import math
import argparse
from pymavlink import mavutil

# TODO: sửa lại cách lấy tọa độ, cách di chuyển, vận tốc, thời gian

# TODO: chuyển thành biến hằng #done
SENSOR_HEIGHT = 8.88 * 0.001  # chiều cao sensor (mét) theo tỉ lệ 3:2
SENSOR_WIDTH = 13.31 * 0.001  # chiều rộng sensor (mét) theo tỉ lệ 3:2
IMAGE_HEIGHT = 1080  # độ cao của ảnh (pixel)
IMAGE_WIDTH = 1620  # độ rộng của ảnh (pixel)

FOCAL_LENGTH = 0.022/2.8  # tiêu cự camera (mét)
ARUCO_IDS = [100, 95, 90, 85]  # các id của aruco
ARUCO_SIZE = [0.375, 0.155, 0.09, 0.075]  # kích thước thật của aruco (mét)

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
PIXEL_SIZE_IN_METER = SENSOR_HEIGHT / IMAGE_HEIGHT  # kích thước 1 pixel (mét) trên ảnh


aruco_dictionary = aruco.getPredefinedDictionary(
    aruco.DICT_4X4_250
)  # DICT_4X4_250 can be changed
aruco_parameters = aruco.DetectorParameters()
aruco_detector = aruco.ArucoDetector(aruco_dictionary, aruco_parameters)


class Vector:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

def detect_aruco(image):
    # TODO: thêm chú thích mục đích, input, output
    # tương tự cho hàm khác
    """
    Input: image
    Output: coordinate and id of each aruco
    Purpose: get coordinates (pixel) and ids of aruco codes in image
    """
    corners, ids, rejected = aruco_detector.detectMarkers(image)  # TODO: reject -> _
    return corners, ids



# # TODO sửa lại cách lấy tọa đ
def get_coordinate(image):
    """
    Input: image
    Output: coordinate of landing pad and altitude of drone
    Purpose: get coordinate of landing pad in meter and calculate altitude of drone
    """
    conners, ids = detect_aruco(image)
    if ids is None:
        return None
    ids = list(ids)
    id = ids[0][0]
    corner = conners[0][0]
    pad = PAD(id, corner)
    x = int(pad.x)
    y = int(pad.y)
    
    height_aruco = math.sqrt(
        (conners[0][0][1][0] - conners[0][0][0][0])**2 + (conners[0][0][1][1] - conners[0][0][0][1])**2
    )  # chiều cao của aruco (pixel) trên ảnh
    height_aruco += math.sqrt(
        (conners[0][0][2][0] - conners[0][0][1][0])**2 + (conners[0][0][2][1] - conners[0][0][1][1])**2
    )
    height_aruco += math.sqrt(
        (conners[0][0][3][0] - conners[0][0][2][0])**2 + (conners[0][0][3][1] - conners[0][0][2][1])**2
    )
    height_aruco += math.sqrt(
        (conners[0][0][0][0] - conners[0][0][3][0])**2 + (conners[0][0][0][1] - conners[0][0][3][1])**2
    )
    height_aruco = height_aruco/4
    height_aruco = (
        height_aruco * PIXEL_SIZE_IN_METER
    )  # chiều cao tương ứng của aruco (mét) trên ảnh
    h = 0
    for id in ids:
        if id in ARUCO_IDS:
            index = ARUCO_IDS.index(ids[0])
            h_aruco = ARUCO_SIZE[
                index
            ] 

            h = h_aruco * FOCAL_LENGTH / height_aruco
            break
    return (x, y), h


def get_vector_movement(image, h0=0):
    """
    Input: image
    Output: vector from drone to landing pad
    Purpose: calculate vector in meter to control drone
    """
    
    coord, h = get_coordinate(image)
    r = h / FOCAL_LENGTH  # tỉ lệ quy đổi
    y_center = (IMAGE_HEIGHT - 1) / 2  # tọa độ tâm của màn ảnh theo trục y (pixel)
    x_center = (IMAGE_WIDTH - 1) / 2  # tọa độ tâm của màn ảnh theo trục x (pixel)
    pixel_size = PIXEL_SIZE_IN_METER * r  # kích thước của 1 pixel trên thực tế
    x_aruco, y_aruco = coord  # tọa độ tâm của pad
    x = (
        x_aruco - x_center
    )  # khoảng cách từ tâm của drone đến tâm của pad theo x (pixel)
    y = (
        y_aruco - y_center
    )  # khoảng cách từ tâm của drone đến tâm của pad theo y (pixel)

    
    x *= pixel_size  # quy đổi khoảng cách ra mét
    y *= pixel_size  # quy đổi khoảng cách ra mét

    return x, y, -h + h0

# def get_yaw(vector_pad):
#     yaw = np.arctan(-vector_pad/vector_pad.y)*(180/np.pi)
#     return yaw

# #TODO controller

# # TODO: gộp 2 hàm và đổi tên thành calculate_velocity #done
def calculate_verticalThrottle(
    distance_z,
    max_distance=MAX_DISTANCE_Z,
    min_distance=MIN_DISTANCE_Z,
    max_speed=MAX_SPEED_Z,
    min_speed=MIN_SPEED_Z,
):
    """
    purpose: estimate verticalThrottle base on altitude of drone
    """
    if distance_z > max_distance:
        drone_speed_z = max_speed
    elif distance_z < min_distance:
        drone_speed_z = min_speed
    else:
        drone_speed_z = max_speed * (distance_z / max_distance)
    return drone_speed_z



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
    return pitch

#cắm đầu xuống dưới

# arm_and_takeoff(0.6)
# print('cmcmcmcmc')

# vehicle.mode = VehicleMode("LAND")

# def convert_to_drone_controller(
#     vector_movement,
#     threshold_x=THRESHOLD_X,
#     threshold_yaw=THRESHOLD_YAW,
# ):
#     """
#     convert from descartes coordinate to parameter of drone controller
#     """
#     x, y, z = vector_movement
#     distance_x = math.sqrt(x**2 + y**2)
#     distance_z = abs(z)
#     landing_time = estimate_landing_time_x(distance_x) + estimate_landing_time_z(
#         distance_z
#     )
#     # TODO nếu landing_time lớn hơn thời gian pin còn lại --> ???

#     rotated_angle = math.atan2(y, x)
#     rotated_angle = rotated_angle * (180 / math.pi)  # Convert radian to degree
#     yaw = get_yaw(rotated_angle)
#     if abs(yaw) >= threshold_yaw:
#         return get_param_while_spin_around(yaw), landing_time + 1

#     if distance_x <= threshold_x:
#         return get_param_vertical_flight(distance_z), landing_time
#     else:
#         return get_param_while_fly_horizontally(distance_x, y, yaw), landing_time


# def estimate_landing_time_z(
#     distance_z,
#     max_distance_z=MAX_DISTANCE_Z,
#     min_distance_z=MIN_DISTANCE_Z,
#     max_speed_z=MAX_SPEED_Z,
#     min_speed_z=MIN_SPEED_Z,
# ):
#     k = MAX_SPEED_X / MAX_DISTANCE_X
#     """
#     ước tính thời gian hạ cánh, để kiểm soát trường hợp ngoại lệ
#     Ví dụ: Thời gian hạ cánh vượt quá thời gian còn lại của pin
#     """
#     # TODO: giải thích công thức #done
#     # 2(distance_x - min_distance_x) / (drone_speed_x + min_speed_x)
#     if distance_z < min_distance_z:
#         moving_time_z = distance_z / min_speed_z
#     elif min_distance_z < distance_z < max_distance_z:
#         moving_time_z = min_distance_z / min_speed_z
#         +math.log((min_distance_z / distance_z), (1 - k))
#     else:
#         moving_time_z = (distance_z - max_distance_z) / max_speed_z
#         +math.log(min_distance_z / max_distance_z, (1 - k))
#         +min_distance_z / min_speed_z

#     return moving_time_z


# def estimate_landing_time_x(
#     distance_x,
#     max_distance_x=MAX_DISTANCE_X,
#     min_distance_x=MIN_DISTANCE_X,
#     max_speed_x=MAX_SPEED_X,
#     min_speed_x=MIN_SPEED_X,
# ):
#     k = MAX_SPEED_X / MAX_DISTANCE_X
#     """
#     ước tính thời gian hạ cánh, để kiểm soát trường hợp ngoại lệ
#     Ví dụ: Thời gian hạ cánh vượt quá thời gian còn lại của pin
#     """
#     # TODO: giải thích công thức #done
#     # 2(distance_x - min_distance_x) / (drone_speed_x + min_speed_x)
#     if distance_x < min_distance_x:
#         moving_time_x = distance_x / min_speed_x
#     elif min_distance_x < distance_x < max_distance_x:
#         moving_time_x = min_distance_x / min_speed_x
#         +math.log(min_distance_x / distance_x, (1 - k))
#     else:
#         moving_time_x = (distance_x - max_distance_x) / max_speed_x
#         +math.log(min_distance_x / max_distance_x, (1 - k))
#         +min_distance_x / min_speed_x

#     return moving_time_x
    # TODO: nếu hạ độ cao quá nhanh so với di chuyến tới trước thì khả năng mất dấu aruco
    # -> chỉnh lại sao cho luôn luôn moving_time_x <= moving_time_z #done


# image = insert_aruco(img1, img2, x1, y1)
# image = cv2.imread("Untitled3.png")
# t = time.time()
# # vector_movement = get_vector_movement(image)

# # control, landing_time = convert_to_drone_controller(vector_movement)
# print(time.time() - t)


# coor, h = get_coordinate(image)

# x, y = coor
# x = int(x)
# y = int(y)
# print(x, y)
# frame = cv2.putText(
#     image,
#     "cccc",
#     (x, y),
#     cv2.FONT_HERSHEY_SIMPLEX,
#     1,
#     (0, 255, 0),
#     2,
#     cv2.LINE_AA,
# )
# frame = cv2.resize(frame, (1920, 1080), interpolation=cv2.INTER_AREA)
# cv2.imshow('dd', frame)
# cv2.waitKey(0)


# vector_movement = get_vector_movement(image)

# control, landing_time = convert_to_drone_controller(vector_movement)

# print(control)
# print(landing_time)
