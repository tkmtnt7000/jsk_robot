#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bs4 import BeautifulSoup
import datetime
import rospy
import subprocess
import sys
if sys.version_info.major == 2:
    import urllib2
else:
    from urllib import request


class RobotLaunchEmail:

    def get_tips(self):
        url = 'https://zatsuneta.com/category/random.html'
        if sys.version_info.major == 2:
            response = urllib2.urlopen(url)
        else:
            response = request.urlopen(url)
        soup = BeautifulSoup(response, 'html5lib')
        topstories = soup.find('div', class_="article")
        title = topstories.find('a')['title']
        detail_url = topstories.find('a')['href']
        contents = topstories.find('p').text
        # rospy.loginfo('Tips title: {}'.format(title))
        # rospy.loginfo('Tips contents: {}'.format(contents))
        response.close()

        return title, contents, detail_url

    def send_mail(self):
        sender_address = "unitreepro@jsk.imi.i.u-tokyo.ac.jp"
        receiver_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"
        current_time = datetime.datetime.now()
        mail_title = "Unitree Go1 が起動しました"
        message = ""
        message += "おはよう。"
        message += "{}時{}分です。\\n".format(
            current_time.hour, current_time.minute)
        message += "\\n"

        contents_title, contents, detail_url = self.get_tips()
        message += "今日の豆知識 \\n"
        # Character code differences between python versions
        # See https://stackoverflow.com/questions/54153986/handling-encode-when-converting-from-python2-to-python3 #NOQA
        if sys.version_info.major == 2:
            message += "{}: {} \\n".format(
                contents_title.encode('utf-8'), contents.encode('utf-8'))
        else:
            message += "{}: {} \\n".format(contents_title, contents)
        message += "詳細: {}".format(detail_url)

        cmd = "echo -e '{}'".format(message)
        cmd += " | /usr/bin/mail -s '{}' -r {} {}".format(
            mail_title, sender_address, receiver_address)
        exit_code = subprocess.call(cmd, shell=True)

        rospy.loginfo('Title: {}'.format(mail_title))
        if exit_code > 0:
            rospy.logerr(
                'Failed to send e-mail:  {} -> {}'.format(
                    sender_address, receiver_address))
            rospy.logerr("You may need to do '$ sudo apt install mailutils'")
        else:
            rospy.loginfo(
                'Succeeded to send e-mail: {} -> {}'.format(
                    sender_address, receiver_address))


if __name__ == '__main__':
    RobotLaunchEmail = RobotLaunchEmail()
    RobotLaunchEmail.send_mail()
