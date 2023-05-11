
from statistics import mean

SCALE_95 = 2.5
SCALE_90 = 25/9
SCALE_85 = 3.066666667



def get_coordinate_center(corner):
    xs = []
    ys = []
    for coor in corner:
        x_aruco, y_aruco = coor
        xs.append(x_aruco)
        ys.append(y_aruco)
    x = mean(xs)
    y = mean(ys)
    return x, y

def calculate_coor_pad_100(corner):
    x_center, y_center = get_coordinate_center(corner)
    x = (x_center + corner[2][0])//2
    y = (y_center + corner[2][1])//2
    return x, y

def calculate_coor_pad_95(corner):
    x_center, y_center = get_coordinate_center(corner)
    x1, y1 = corner[0]
    a = x1 - x_center
    b = y1 - y_center
    x = x_center + 2.5 * a
    y = y_center + 2.5 * b
    return x, y

def calculate_coor_pad_85(corner):
    x_center = (corner[0][0] + corner[1][0])/2
    y_center = (corner[1][1] + corner[0][1])/2
    x4, y4 = corner[3]
    a =  x_center - x4
    b =  y_center - y4
    x = x4 + SCALE_85 * a
    y = y4 + SCALE_85 * b
    return x, y

def calculate_coor_pad_90(corner):
    x_center = (corner[3][0] + corner[0][0])/2
    y_center = (corner[3][1] + corner[0][1])/2
    x2, y2 = corner[1]
    a = x_center - x2
    b = y_center - y2
    x = x2 + a * SCALE_90
    y = y2 + b * SCALE_90
    return x, y

class PAD:
    def __init__(self, id, corner) -> None:
        if id == 100:
            self.x, self.y = calculate_coor_pad_100(corner)
        elif id == 95:
            self.x, self.y = calculate_coor_pad_95(corner)
        elif id == 90:
            self.x, self.y = calculate_coor_pad_90(corner)
        elif id == 85:
            self.x, self.y = calculate_coor_pad_85(corner)
        else:
            self.x, self.y = 0, 0


# img = cv2.imread('Untitled3.png')
# corners, ids = detect_aruco(img)
# id = ids[3][0]
# corner = corners[3][0]
# coor_pad = PAD(id, corner)

# x = int(coor_pad.x)
# y = int(coor_pad.y)
# print(x, y)
# frame = cv2.putText(
#     img,
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