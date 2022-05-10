#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from bs4 import BeautifulSoup
import datetime
import json
import random
import subprocess
import sys
if sys.version_info.major == 2:
    import urllib2
else:
    from urllib import request


class RobotLaunchEmail:
    """
    Send email when robot launched with tips.
    """
    def __init__(self):
        # Character code differences between python versions
        # See https://stackoverflow.com/questions/54153986/handling-encode-when-converting-from-python2-to-python3 #NOQA
        if sys.version_info.major == 2:
            reload(sys)
            sys.setdefaultencoding('utf-8')
        api_key_file = '/var/lib/robot/openweathermap_api_key.txt'
        with open(api_key_file, 'r') as f:
            self.appid = f.read().split('\n')[0]

    def get_tips(self):
        """
        Get tips from zatsuneta random article.

        Returns:
        ----------
        title : str
            article title
        contents : str
            article contents (heading)
        detail_url :str
            URL of the article written about the details
        """
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
        response.close()

        return title, contents, detail_url

    # Partly copied from https://github.com/knorth55/jsk_robot/blob/b2999b0cece82d7b1d46320a1e7c84e4fc078bd2/jsk_fetch_robot/jsk_fetch_startup/scripts/time_signal.py #NOQA
    def get_weather_forecast(self, lang='en'):
        url = 'http://api.openweathermap.org/data/2.5/weather?q=tokyo&lang={}&units=metric&appid={}'.format(lang, self.appid)  # NOQA
        if sys.version_info.major == 2:
            resp = json.loads(urllib2.urlopen(url).read())
        else:
            resp = json.loads(request.urlopen(url).read())
        weather = resp['weather'][0]['description']

        forecast_text = ""
        if lang == 'ja':
            forecast_text = "今日の天気は" + weather + "です。"
        else:
            forecast_text = " The weather is " + weather + " now."

        # It feels like the robot is expressing its will based on the weather.
        if "晴" in weather:
            forecast_tuple = (
                '日差しに気をつけて。',
                'お散歩しよう！',
            )
        elif "雨" in weather:
            forecast_tuple = (
                '部屋の中で遊ぼう！',
                '傘忘れていない？',
            )
        elif "雲" or "曇" or "くもり" in weather:
            forecast_tuple = (
                '晴れたらいいな',
            )
        elif "雪" in weather:
            forecast_tuple = (
                '雪合戦しよう！',
                '寒さに気をつけて',
            )
        else:
            forecast_tuple = (' ')
        forecast_text += random.choice(forecast_tuple)

        return forecast_text

    def send_mail(self):
        """
        Send mail with mailutils
        """
        sender_address = "unitreepro@jsk.imi.i.u-tokyo.ac.jp"
        receiver_address = "unitree@jsk.imi.i.u-tokyo.ac.jp"
        current_time = datetime.datetime.now()
        mail_title = "Unitree Go1 が起動しました"
        message = ""
        message += "おはよう。"
        message += "{}時{}分です。\\n".format(
            current_time.hour, current_time.minute)
        message += "{} \\n".format(self.get_weather_forecast(lang="ja"))
        message += "\\n"

        contents_title, contents, detail_url = self.get_tips()
        message += "今日の豆知識 \\n"
        message += "{}: {} \\n".format(contents_title, contents)
        message += "詳細: {} \\n".format(detail_url)
        message += "\\n"

        # echo -e option is necessary in raspberry pi
        cmd = "echo -e '{}'".format(message)
        cmd += " | /usr/bin/mail -s '{}' -r {} {}".format(
            mail_title, sender_address, receiver_address)
        exit_code = subprocess.call(cmd, shell=True)


if __name__ == '__main__':
    RobotLaunchEmail = RobotLaunchEmail()
    RobotLaunchEmail.send_mail()
