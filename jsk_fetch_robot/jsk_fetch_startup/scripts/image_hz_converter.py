#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image


class ImageHzConverter:
    """
    This class converts image topic hz
    """
    def __init__(self):
        self.topic_hz = rospy.get_param('image_hz', 30)
        self.output = Image()
        self.image_sub = rospy.Subscriber(
            "~input", Image, self.image_callback)
        self.pub = rospy.Publisher(
            "~output", Image, queue_size=1)
        rospy.Timer(
            rospy.Duration(1.0/float(self.topic_hz)),
            self.hz_converter,
            oneshot=False)
        rospy.loginfo('image hz convert starting')

    def image_callback(self, msg):
        self.output = msg

    def hz_converter(self, event):
        self.output.header.stamp = rospy.Time.now()
        self.pub.publish(self.output)


if __name__ == '__main__':
    rospy.init_node('image_hz_converter', anonymous=True)
    image_hz_converter = ImageHzConverter()
    rospy.spin()
