#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import math
import rospy
import subprocess

from jsk_recognition_msgs.msg import PeoplePoseArray


class PeopleNumberNotifier():
    def __init__(self):
        rospy.init_node('get_people_number')
        self.people_sum = 0
        self.people_number = 0
        self.last_publish_time = rospy.Time.now()
        self.last_subscribe_time = rospy.Time.now()
        self.last_calcurate_time = rospy.Time.now()
        self.mail_duration = 180
        rospy.Subscriber("/people_pose", PeoplePoseArray, self.detect_cb)
        # rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.detect_cb)

    def detect_cb(self, msg):
        if rospy.Time.now() - self.last_subscribe_time > rospy.Duration(1):
            self.people_sum += len(msg.poses)
            self.last_subscribe_time = rospy.Time.now()
            rospy.loginfo("length:{}".format(len(msg.poses)))
            rospy.loginfo("sum:{}".format(self.people_sum))
        if rospy.Time.now() - self.last_calcurate_time > rospy.Duration(5):
            self.people_number += math.ceil(self.people_sum / 5.0)
            self.people_sum = 0
            self.last_calcurate_time = rospy.Time.now()
            rospy.loginfo("number:{}".format(self.people_number))

    def send_mail(self):
        """
        Send mail with mailutils
        """
        sender_address = "unitreepro@jsk.imi.i.u-tokyo.ac.jp"
        receiver_address = "unitree@jsk.imi.i.u-tokyo.ac.jp"
        # receiver_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"
        mail_title = "お散歩中に人と出会いました．"
        if self.people_number == 0:
            message = "誰とも出会わなかったよ．"
        else:
            message = "{}人の人たちと出会ったよ．".format(
                int(math.ceil(self.people_number / (self.mail_duration / 5.0))))
        print(message)
        cmd = "echo -e '{}'".format(message)
        cmd += " | /usr/bin/mail -s '{}' -r {} {}".format(
            mail_title, sender_address, receiver_address)
        subprocess.call(cmd, shell=True)
        self.people_number = 0
        self.last_publish_time = rospy.Time.now()

    def spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if rospy.Time.now() - self.last_publish_time > \
               rospy.Duration(self.mail_duration):
                try:
                    self.send_mail()
                except Exception as e:
                    rospy.logerr("{}".format(e))
                    continue
                rospy.loginfo('send mail. Waiting...')

    # def run(self):
    #     seconds = 300
    #     while True:
    #         try:
    #             self.send_mail
    #         except Exception as e:
    #             print('error')
    #             print(e)
    #             continue
    #         print('send')
    #         print('waiting')
    #         time.sleep(seconds)


if __name__ == '__main__':
    get_people_number = PeopleNumberNotifier()
    get_people_number.spin()
