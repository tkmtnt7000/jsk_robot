#!/usr/bin/python

# Partly copied from https://github.com/jsk-ros-pkg/jsk_robot/commit/af53a852a0fda5a4fa74fd5656644e2b9ec6bd27#diff-230418b540ef3899fe419fb7c69a262fe5f90317215d1493705eb1b105580139 #NOQA

import base64
import numpy as np
import pickle
import rospy
import sys

# date
from datetime import datetime
from datetime import timedelta
from datetime import tzinfo
import calendar
import pytz

# mongo
from mongodb_store.message_store import MessageStoreProxy
import mongodb_store.util as MU
import pymongo

# message
from jsk_robot_startup.msg import Email
from jsk_robot_startup.msg import EmailBody
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from smach_msgs.msg import SmachContainerStatus

# image processing
from cv_bridge import CvBridge
import cv2

# global variabels
from dateutil import tz
JST = tz.gettz('Asia/Tokyo')
print(JST)
# class JST(tzinfo):
#     def utcoffset(self, dt): return timedelta(minutes=-399)
# JST = pytz.timezone('Asia/Tokyo')
bridge = CvBridge()


def cb(msg):
    if msg.data == "最近どう":
        _get_latest_image(_query_latest_smach())
    elif msg.data == "やかん見た":  # kettle
        _get_latest_object_recognition(label_name="kettle")
    else:
        return
    _send_mail(subject, image)


def _get_latest_image(imgmsg):
    if imgmsg is not None:
        rospy.loginfo("get latest image")
    else:
        rospy.logerr("unable to get latest image")
    # show data..
    # for msg in imgmsg:
    #     print(" @{}".format(
    #         datetime.fromtimestamp(msg.header.stamp.to_sec(), JST)))
    #     cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    #     cv2.imshow('image', cv_image)
    #     cv2.waitKey(5)


def _query_latest_smach():
    '''
    Get latest image stored in the form of smach data from database
    '''
    # image = None
    try:
        rospy.loginfo("Loading last smach execution...")
        last_msg, _ = msg_store.query(
            SmachContainerStatus._type,
            {"info": "START"},
            single=True,
            sort_query=[("_meta.inserted_at", pymongo.DESCENDING)]
        )
        msgs = msg_store.query(
            SmachContainerStatus._type,
            {"header.stamp.secs": {"$gt": last_msg.header.stamp.secs}},
            sort_query=[("_meta.inserted_at", pymongo.ASCENDING)]
        )

        def status_to_img(msg):
            if sys.version_info.major < 3:
                local_data_str = pickle.loads(msg.local_data)
            else:
                local_data_str = pickle.loads(
                    msg.local_data.encode('utf-8'), encoding='utf-8')
            print("{} @{}".format(
                local_data_str['DESCRIPTION'],
                datetime.fromtimestamp(msg.header.stamp.to_sec(), JST)
            ))
            imgmsg = None
            # if local_data_str.has_key('IMAGE') and local_data_str['IMAGE']:
            if 'IMAGE' in local_data_str and local_data_str['IMAGE']:
                imgmsg = CompressedImage()
                imgmsg.deserialize(base64.b64decode(local_data_str['IMAGE']))
            return imgmsg

        return filter(
            lambda x: x is not None, map(lambda x: status_to_img(x[0]), msgs))
    except Exception as e:
        rospy.logerr('failed to load images from db: %s' % e)
    return None


def _get_latest_object_recognition(
        label_name=None,
        now=datetime.now(JST)-timedelta(hours=0),
        then=datetime.now(JST)-timedelta(hours=1)
):
    '''
    if label_name is in database
    return  latest object recognition result from database
    '''
    image = None
    return image


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
db_name = 'jsk_robot_lifelog'
col_name = 'basil'  # pr1012, fetch17 etc..
msg_store = MessageStoreProxy(database=db_name, collection=col_name)
