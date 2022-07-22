#!/usr/bin/env python

import numpy as np
import rospy
import pickle
import base64
import imghdr
from io import StringIO
from io import BytesIO

from smach_msgs.msg import SmachContainerStatus

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class SmachToMail():

    def __init__(self):
        rospy.init_node('server_name')  # it should be 'smach_to_mail', but 'server_name' is the default name of smach_ros
        rospy.Subscriber(
            "~smach/container_status", SmachContainerStatus, self._status_cb)
        # rospy.Subscriber(
        #     "/usb_cam/image_raw", Image, self._img_cb)
        self.bridge = CvBridge()
        self.dict_list = []

    # def _img_cb(self, msg):
    #     buff = StringIO()
    #     # buff = BytesIO()
    #     msg.serialize(buff)
    #     # tmpmsg = base64.b64encode(buff)
    #     # imgmsg = Image()
    #     # imgmsg.deserialize(base64.b64decode(tmpmsg))

    def _status_cb(self, msg):
        if len(msg.active_states) == 0:
            return
        file_path = None
        status_str = ', '.join(msg.active_states)
        local_data_str = pickle.loads(msg.local_data)
        info_str = msg.info
        print("status -> {}".format(status_str))
        print("description_str -> {}".format(local_data_str['DESCRIPTION']))
        # print("image_str -> {}".format(local_data_str['IMAGE']))
        if local_data_str['IMAGE']:
            imgmsg = CompressedImage()
            # print(base64.b64decode(local_data_str['IMAGE']))
            imgmsg.deserialize(base64.b64decode(local_data_str['IMAGE']))
            # imgmsg.deserialize(base64.b64decode(local_data_str['IMAGE']))
            cv_image = self.bridge.compressed_imgmsg_to_cv2(imgmsg, "bgr8")
            # rospy.loginfo("compressd image type:{}".format(imghdr.what(None, imgmsg)))
            rospy.loginfo("cv image type:{}".format(imghdr.what(None, cv_image)))
            # np_arr = np.fromstring(imgmsg.data, np.uint8)
            # input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_image = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
            cv2.imshow("Image", cv_image)
            file_path = "/tmp/{}.jpg".format(status_str)
            rospy.loginfo("filepath:{}".format(file_path))
            if (next((x for x in self.dict_list if x["IMAGE"] == file_path), None)):
                rospy.loginfo("same file name!!!!")
            else:
                rospy.loginfo("not same file name!!!")
            cv2.imwrite(file_path, cv_image)
            # cv2.imshow("Image", input_image)
            cv2.waitKey(2)
        tmp_dict = {'DESCRIPTION': local_data_str['DESCRIPTION'],
                    'IMAGE': file_path}
        self.dict_list.append(tmp_dict)
        print(self.dict_list)
        print("info_str -> {}".format(info_str))


stm = SmachToMail()
rospy.spin()
