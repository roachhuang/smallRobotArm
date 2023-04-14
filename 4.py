from time import sleep
import init_serial as com
import robotarm_class as robot
import freenect
import numpy as np
import cv2


# pose of camera frame measured from robot frame.
R_RC = np.array([
    [1, 0, 0],
    [0,  -1, 0],
    [0,  0, -1]])

T_RC = np.eye(4)   # identity matrix, placeholder
T_RC[:3, :3] = R_RC[:3, :3]
# T_RC[:3, 3] = (350, 65, 625)
T_RC[:3, 3] = (440, 1, 625)
print(T_RC)


def depth_to_3d(x, y, depth):
    ''' retruned values are in mm '''
    fx = 5.7616540758591043e+02
    fy = 5.7375619782082447e+02
    cx = 3.1837902690380988e+02
    cy = 2.3743481382791673e+02

    # z is in mm
    z = depth[y, x]
    # z=500   # 40cm
    x_ = (x - cx) * z / fx
    y_ = (y - cy) * z / fy

    return (x_, y_, z)


run = True
a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0
std_dh_params = np.array([
    [np.radians(-90), a1, d1], [0, a2, 0], [np.radians(-90), a3, 0],
    [np.radians(90), 0, d4], [np.radians(-90), 0, 0], [0, 0, d6]
])

smallRobotArm = robot.smRbtArm(std_dh_params)
# initialize the serial connection to the Arduin
ser = com.init_ser()
# there must be a dealy here!!!
sleep(1)
ser.write(b"en\n")
sleep(.5)
ser.write(b"rst\n")
sleep(.5)


while run == True:
    # Get depth and RGB frames from Kinect
    depth, _ = freenect.sync_get_depth()
    bgr, _ = freenect.sync_get_video()
    flipped_bgr = cv2.flip(bgr, -1)
    h, w = depth.shape
    #cx = int(width/2)
    #cy= int(height/2)

    rgb = cv2.cvtColor(flipped_bgr, cv2.COLOR_BGR2RGB)

    cv2.line(rgb, (int(w/2), 0), (int(w/2), h), (0, 255, 0), 1)
    cv2.line(rgb, (0, int(h/2)), (w, int(h/2)), (0, 255, 0), 1)
    cv2.circle(rgb, (int(w/2), int(h/2)), 10, (0, 255, 0), 1)

    blurred = cv2.GaussianBlur(flipped_bgr, (11, 11), 0)
    hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

    lower_red = np.array([161, 155, 84])
    upper_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    k = np.ones((3, 3))
    eroded = cv2.erode(red_mask, k, iterations=1)
    dilated = cv2.dilate(eroded, k, iterations=7)
    # cnts, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cnts, _ = cv2.findContours(
        dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area = cv2.contourArea(c)
        if area > 1000:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.circle(rgb, (cX, cY), 7, (255, 255, 255), 2)
            x, y, z = depth_to_3d(cX, cY, depth)
            T_CO = robot.pose2T([x, y, z, 0, 0, 35])
            T_RO = T_RC @ T_CO
            obj_position = T_RO[0:3, 3]
            obj_zyz = robot.euler_zyz_from_matrix(T_RO)
            #obj_zyz_in_rad = np.array(obj_zyz)
            obj_zyz_in_rad = np.array([0, 0, 0])
            obj_pose = np.concatenate(
                (np.array(obj_position), np.degrees(obj_zyz_in_rad)))
            obj_pose = np.around(obj_pose, decimals=2)
            print(obj_pose)

            cv2.putText(rgb, 'x:{:.2f}, y:{:.2f}, z:{:.2f}'.format(x, y, z), (cX - 20,
                                                                              cY - 20), cv2.FONT_ITALIC, 1, (255, 0, 0), 1, cv2.LINE_AA)

            cv2.drawContours(rgb, cnts, -1, (0, 255, 0), 2)

            try:
                j = smallRobotArm.ik(obj_pose)
                msg = '{}{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(
                    'j', *j,)
                ser.write(msg.encode('utf-8'))
                run = False
            except:
                print('ik error!')

    cv2.imshow("img", rgb)
    # cv2.imshow("msk", red_mask)

    if cv2.waitKey(10) & 0xff == ord('q'):
        break

cv2.destroyAllWindows()
