#!/usr/bin/env python

import datetime
# import numpy as np
import rospy
import pickle
import base64
import imghdr
# from io import StringIO
# from io import BytesIO

from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody
from smach_msgs.msg import SmachContainerStatus

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class SmachToMail():

    def __init__(self):
        rospy.init_node('server_name')  # it should be 'smach_to_mail', but 'server_name' is the default name of smach_ros
        rospy.Subscriber(
            "~smach/container_status", SmachContainerStatus, self._status_cb)
        self.bridge = CvBridge()
        self.dict_list = [] # for status list
        self.sender_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"
        self.receiver_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"
        
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
            dt_now = datetime.datetime.now()
            file_path = "/tmp/{}_{}.jpg".format(
                status_str.lower(), dt_now.strftime('%y%m%d%H%M%S'))
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

        # if msg.active_states = "END":
        #     self._send_mail()

    def _send_mail():
        email_msg = Email()
        content1 = EmailBody()
        content1.type = 'text'
        content2 = EmailBody()
        content2.type = 'text'
        changeline = EmailBody()
        changeline.type = 'html'
        changeline.message = "<br>"
        
        content3 = EmailBody()
        content3.type = 'img'
        content3.file_path = "/tmp/hoge.jpg"
        content3.img_size = 50
        # print(msg.active_states)
        # print(msg.local_data)
        content1.message = msg.active_states[0]
        rospy.loginfo("active state: {}".format(content1.message))
        local_data = pickle.loads(msg.local_data)
        content2.message = "".join(local_data)
        # content2.message = msg.local_data
        rospy.loginfo("local data: {}".format(content2.message))

        email_msg.body = [content1, changeline, content2, changeline, content3]
        email_msg.subject = 'Fetch Go to Kitchen Smach'

        email_msg.sender_address = self.sender_address
        email_msg.receiver_address = self.receiver_address

        time.sleep(1)
        self.pub.publish(email_msg)


stm = SmachToMail()
rospy.spin()
