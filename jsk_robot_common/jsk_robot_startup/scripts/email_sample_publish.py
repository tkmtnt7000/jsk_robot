#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody

def main():
    rospy.init_node('embed_image_test')
    pub = rospy.Publisher("email", Email, queue_size=10)

    msg = Email()
    content_1 = EmailBody()
    content_1.type = 'text'
    content_1.message = 'こんにちは．\n'
    content_2 = EmailBody()
    content_2.type = 'img'
    content_2.file_path = '/home/tsukamoto/Pictures/detic.jpg'
    content_3 = EmailBody()
    content_3.type = 'audio'
    content_3.file_path = '/home/tsukamoto/sample.wav'

    msg.body = [content_1, content_3]
    msg.subject = 'embed_test'
    # msg.body = [content_1]
    msg.sender_address = 'tsukamoto@jsk.imi.i.u-tokyo.ac.jp'
    msg.receiver_address = 'tsukamoto@jsk.imi.i.u-tokyo.ac.jp'
    print (msg)

    time.sleep(1)
    rospy.loginfo("Publish")
    pub.publish(msg)

main()
