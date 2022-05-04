#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from urllib import request
from bs4 import BeautifulSoup
import datetime
import subprocess


def web():
    url = 'https://zatsuneta.com/category/random.html'
    response = request.urlopen(url)
    soup = BeautifulSoup(response, 'html5lib')

    topstories = soup.find('div', class_="article")
    title = topstories.find('a')['title']
    detail_url = topstories.find('a')['href']
    contents = topstories.find('p').text
    print(title)
    print(contents)
    response.close()

    return title, contents, detail_url

def email():
    t_delta = datetime.timedelta(hours=9)
    JST = datetime.timezone(t_delta, 'JST')
    current_time = datetime.datetime.now(JST)

    message = ""
    message += "おはよう。"
    # if current_time.hour < 4:
    #     message += "こんばんは。"
    # elif current_time.hour < 10:
    #     message += "おはよう。"
    # elif current_time.hour < 17:
    #     message +=  "こんにちは。"
    # else:
    #     message += "こんばんは。"

    message += "{}時{}分です。\\n".format(
        current_time.hour, current_time.minute)
    message += "\\n"
    contents_title, contents, detail_url = web()
    message += "今日の豆知識 \n"
    message += "{}: {} \n".format(contents_title, contents)
    message += "詳細: {}".format(detail_url)
    cmd = "echo -e '{}' | mail -s 'Unitree Go1 が起動しました' -r unitreepro@jsk.imi.i.u-tokyo.ac.jp tsukamoto@jsk.imi.i.u-tokyo.ac.jp".format(message)
    # cmd = "echo -e '{}' | mail -s 'Unitree Go1 が起動しました' -r unitreepro@jsk.imi.i.u-tokyo.ac.jp yanokura@jsk.imi.i.u-tokyo.ac.jp".format(message)
    subprocess.call(
        cmd, shell=True)

email()
