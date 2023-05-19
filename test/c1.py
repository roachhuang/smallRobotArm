"""
Object detection: The first step is to detect the object in the scene. This can be done using techniques such as object recognition and segmentation. Object recognition involves identifying the object based on its appearance, while segmentation involves separating the object from the background based on its shape and color.

Object localization: Once the object is detected, the next step is to determine its position and orientation with respect to the robot. This can be done using techniques such as pose estimation, which involves estimating the pose of the object in 3D space based on its appearance in 2D images.

Grasp planning: After the object is localized, the robot needs to plan a grasp that is both safe and efficient. This involves selecting the appropriate grasp type, such as a parallel-jaw or a suction-cup grasp, and determining the optimal contact points on the object for the selected grasp. The robot also needs to consider factors such as the object's weight, shape, and surface properties, as well as any obstacles in the environment.

Grasp execution: Once the grasp is planned, the robot needs to execute the grasp. This involves moving the end-effector, which is typically a robotic hand, to the planned contact points and applying the appropriate forces to grasp the object. The robot also needs to ensure that the grasp is stable and that the object is not dropped during manipulation.

Overall, grasping an object is a complex task that involves many different components, including computer vision, motion planning, and control. To perform this task effectively, robots need to be equipped with advanced sensors and algorithms that enable them to perceive and interact with the environment in a robust and reliable manner.

""""""
import cv2

cap = cv2.VideoCapture(0)  # 开始读取摄像头信号
pointlist = []  # 声明一个列表用于存储点的位置
start = 0  # 声明一个变量表示是否开始记录点的位置
while cap.isOpened():  # skip 60 frames unstable reading from camera
    for i in range(60):
        (ret, frame) = cap.read()  # 读取每一帧视频图像为frame

while cap.isOpened():  # 当读取到信号时
    (ret, frame) = cap.read()  # 读取每一帧视频图像为frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将颜色空间转换为HSV
    #yellow_lower = (26, 43, 46)  # 指定目标颜色的下限
    #yellow_upper = (34, 255, 255)  # 指定目标颜色的上限
    black_lower = (0, 0, 0)
    black_upper=(180, 255, 46)
    mask = cv2.inRange(hsv, black_lower, black_upper)  # 使用目标范围分割图像并二值化
    (cnts, hierarchy) = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)  # 寻找其中的所有外轮廓
    if len(cnts) > 0:  # 如果至少找到一个轮廓
        c = max(cnts, key=cv2.contourArea)  # 找出其中面积最大的轮廓
        ((x, y), radius) = cv2.minEnclosingCircle(c)  # 分析轮廓的中心位置和大小
        if radius > 50:  # 仅当半径大于20 时
            print(" 中心坐标", (x, y))
            if start == 1:  # start = 1 说明开始记录
                pointlist.append([x, y])  # 将点的位置追加到pointlist 列表
            elif start == 0:  # start = 0 说明需要清除图像上的点
                pointlist = []  # 将pointlist 重新置空
    for point in pointlist:  # 遍历pointlist 中所有点的位置
        x = point[0]
        y = point[1]
        # 在这些点的位置上绘制一个彩色小圆点
        cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 255), -1)
    cv2.imshow('MagicWand', frame)  # 将图像显示到屏幕上
    k = cv2.waitKey(5)  # 每一帧后等待5 毫秒，并将键盘的按键值存为k
    # 如果按q 键，程序退出；按s 键，开始绘制；按p 键，停止绘制；按e 键，清除绘制
    if k == ord("q"):
        break
    elif k == ord("s"):
        start = 1
    elif k == ord("p"):
        start = -1
    elif k == ord("e"):
        start = 0


    #    print(" 中心坐标", (x, y))
    #    print(" 半径", radius)
    #cv2.imshow('test', frame)  # 将图像显示到屏幕上
    #cv2.waitKey(5)  # 每一帧后等待5 毫秒
