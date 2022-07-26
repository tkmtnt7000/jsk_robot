#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from bs4 import BeautifulSoup
import datetime
import random
import subprocess
import sys
import time
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
        mail_title = "Unitree Go1 がお散歩しています"
        message = ""
        message += "{}時{}分です。\\n".format(
            current_time.hour, current_time.minute)
        message += "\\n"

        walking_message = (
            'お散歩楽しいな！',
            'お散歩気持ちいいね！'
            'たくさん人がいるね',
            'そろそろ帰りたくなってきたな',
        )
        message += random.choice(walking_message)

        cmd = "echo -e '{}'".format(message)
        cmd += " | /usr/bin/mail -s '{}' -r {} {}".format(
            mail_title, sender_address, receiver_address)
        subprocess.call(cmd, shell=True)


if __name__ == '__main__':
    RobotLaunchEmail = RobotLaunchEmail()
    while True:
        RobotLaunchEmail.send_mail()
        time.sleep(300)
