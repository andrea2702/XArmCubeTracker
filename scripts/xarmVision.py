#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from xarm_msgs.srv import *

# set fps
fps = 10

# set controller constants
kp_linx = 0.0234375*2
kp_liny = 0.03125*2
kp_angz = 0.25

# set hsv mask lower and upper bounds
lower = np.array([100, 200, 50], dtype=np.uint8)
upper = np.array([125, 255, 255], dtype=np.uint8)


# set stop sequence to halt the robot
def stop():
    cv2.destroyAllWindows()
    cap.release()
    move_line([0,0,0,0,0,0], 0, 1)


if __name__ == '__main__':
    # init camera and node
    cap = cv2.VideoCapture(2)
    rospy.init_node("xarm_visservo")
    rospy.on_shutdown(stop)

    # wait for service
    rospy.wait_for_service('/xarm/velo_move_line')
    rospy.wait_for_service('/xarm/move_joint')

    # get xArm service proxies
    move_line = rospy.ServiceProxy('/xarm/velo_move_line', MoveVelo)
    motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
    set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
    set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
    get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
    home = rospy.ServiceProxy('/xarm/go_home', Move)
    move_joint = rospy.ServiceProxy('/xarm/move_joint', Move)
    rate = rospy.Rate(fps)

    # set robot to starting configuration
    try:
        motion_en(8,1)
        set_mode(0)
        set_state(0)
        ret = move_joint([0,-0.6806,-0.5934,0,1.2915,0],0.35,7,0,0)

    except rospy.ServiceException as e:
        print("Before servo_cartesian, service call failed: %s" % e)
        exit(-1)

    # set robot to move using cartesian velocities
    set_mode(5)
    set_state(0)

    # main loop
    while not rospy.is_shutdown():
        # read image
        ret, img = cap.read()
        if not ret:
            break
        rot_bbox = img.copy()
        
        # get color mask to only track the color blue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        img2 = img.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img2 = cv2.bitwise_and(img, img, mask=mask)
        
        # get a threshold to only get high blue values
        gray = 255-img2
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 75, 2)
        thresh = 255 - thresh

        # apply morphology
        kernel = np.ones((5, 5), np.uint8)
        rect = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        rect = cv2.morphologyEx(rect, cv2.MORPH_CLOSE, kernel)

        # thin
        kernel = np.ones((5, 5), np.uint8)
        rect = cv2.morphologyEx(rect, cv2.MORPH_ERODE, kernel)

        # get largest contour
        contours = cv2.findContours(rect, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        big_contour = None
        for c in contours:
            area_thresh = 0
            area = cv2.contourArea(c)
            if area > area_thresh:
                area = area_thresh
                big_contour = c

        # get rotated rectangle from contour
        if big_contour is not None:
            rot_rect = cv2.minAreaRect(big_contour)
            box = cv2.boxPoints(rot_rect)
            box = np.int0(box)
            if box[0][0] > box[2][0]:
                box = np.array([box[1], box[2], box[3], box[0]])
            cv2.drawContours(rot_bbox, [box], 0, (0, 0, 255), 2)

            # calculate errors and move xArm
            error = np.array([[320, 240],[320, 240],[320, 240],[320, 240]]) - box
            error = error.sum(axis=0)
            theta_error = np.arctan2((box[2][1]-box[1][1]), (box[2][0] - box[1][0]))
            theta_error = -np.arctan2(np.sin(theta_error), np.cos(theta_error))
            z_error = 30*np.log(10000/cv2.contourArea(big_contour))
            move_line([error[1]*kp_liny, -error[0]*kp_linx, z_error, 0, 0, -theta_error*kp_angz], 0, 1)
        else:
            print("no u")
            move_line([0, 0, 0, 0, 0, 0], 0, 1)
        
        # show image on computer
        cv2.imshow("filter", gray)
        cv2.imshow("box", rot_bbox)
        # print(box.shape)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
        rate.sleep()

