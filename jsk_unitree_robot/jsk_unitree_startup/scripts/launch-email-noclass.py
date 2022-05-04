#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bs4 import BeautifulSoup
import datetime
import subprocess
import sys
if sys.version_info.major == 2:
    import urllib2
else:
    from urllib import request

# without this, the following error occurs
# UnicodeEncodeError: 'ascii' codec can't encode characters in position 0-9: ordinal not in range(128) #NOQA
# sys.stdout = codecs.getwriter('utf_8')(sys.stdout)
# sys.stdin = codecs.getreader('utf_8')(sys.stdin)

def web():
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
    print(title)
    print(contents)
    response.close()

    return title, contents, detail_url

def email():
    # t_delta = datetime.timedelta(hours=9)
    # JST = datetime.timezone(t_delta, 'JST')
    current_time = datetime.datetime.now()

    mail_title = "Unitree Go1 が起動しました"
    sender_address = "unitreepro@jsk.imi.i.u-tokyo.ac.jp"
    receiver_address = "tsukamoto@jsk.imi.i.u-tokyo.ac.jp"
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
    message += "今日の豆知識 \\n"
    if sys.version_info.major == 2:
        message += "{}: {} \\n".format(
            contents_title.encode('utf-8'), contents.encode('utf-8'))
    else:
        message += "{}: {} \\n".format(contents_title, contents)
    message += "詳細: {}".format(detail_url)

    cmd = "echo -e '{}' ".format(message)
    cmd += " | /usr/bin/mail -s '{}' -r {} {}".format(
        mail_title, sender_address, receiver_address)
    # cmd = "echo -e '{}' | mail -s 'Unitree Go1 が起動しました' -r unitreepro@jsk.imi.i.u-tokyo.ac.jp yanokura@jsk.imi.i.u-tokyo.ac.jp".format(message)
    subprocess.call(
        cmd, shell=True)


if __name__ == '__main__':
    email()
