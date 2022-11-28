#!/usr/bin/python

import rospy
from std_msgs.msg import String
from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody

def cb(msg):
    if msg.data == "最近どう":
        mongodb_function()
        print("hoge")
    elif msg.data == "やかん見た": # kettle
        mongodb_function()
        print("hoge")
    else:
        return
    _send_mail(subject, image)

def mongodb_function():
    return

def _send_mail(subject, image=None):
    email_msg = Email()
    email_msg.body = []
    changeline = EmailBody()
    changeline.type = 'html'
    changeline.message = "<br>"
    separator = EmailBody()
    separator.type = 'text'
    separator.message = "---------------"

    image = EmailBody()
    image.type = 'img'
    image.img_size = 100
    image.img_data = x['IMAGE']
    email_msg.body.append(image)
    email_msg.body.append(changeline)

    pub.publish(email_msg)

rospy.init_node("memory_maker")
pub = rospy.Publisher("/email", Email, 1)
rospy.Subscribe("request", String, cb)
