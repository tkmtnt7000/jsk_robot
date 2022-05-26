#!/usr/bin/env ipython3
# -*- coding:utf-8 -*-

import subprocess
import time
import re

import requests
import cv2
# import nmcli

import sensor_msgs.msg
import rospy
import cv_bridge


cellNumberRe = re.compile(r"^Cell\s+(?P<cellnumber>.+)\s+-\s+Address:\s(?P<mac>.+)$")
regexps = [
    re.compile(r"^ESSID:\"(?P<essid>.*)\"$"),
    re.compile(r"^Protocol:(?P<protocol>.+)$"),
    re.compile(r"^Mode:(?P<mode>.+)$"),
    re.compile(r"^Frequency:(?P<frequency>[\d.]+) (?P<frequency_units>.+) \(Channel (?P<channel>\d+)\)$"),
    re.compile(r"^Encryption key:(?P<encryption>.+)$"),
    re.compile(r"^Quality=(?P<signal_quality>\d+)/(?P<signal_total>\d+)\s+Signal level=(?P<signal_level_dBm>.+) d.+$"),
    re.compile(r"^Signal level=(?P<signal_quality>\d+)/(?P<signal_total>\d+).*$"),
]

# Detect encryption type
wpaRe = re.compile(r"IE:\ WPA\ Version\ 1$")
wpa2Re = re.compile(r"IE:\ IEEE\ 802\.11i/WPA2\ Version\ 1$")

# Runs the comnmand to scan the list of networks.
# Must run as super user.
# Does not specify a particular device, so will scan all network devices.
def scan(interface='wlan0'):
    cmd = ["iwlist", interface, "scan"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    points = proc.stdout.read().decode('utf-8')
    return points


# Parses the response from the command "iwlist scan"
def parse(content):
    cells = []
    lines = content.split('\n')
    for line in lines:
        line = line.strip()
        cellNumber = cellNumberRe.search(line)
        if cellNumber is not None:
            cells.append(cellNumber.groupdict())
            continue
        wpa = wpaRe.search(line)
        if wpa is not None :
            cells[-1].update({'encryption':'wpa'})
        wpa2 = wpa2Re.search(line)
        if wpa2 is not None :
            cells[-1].update({'encryption':'wpa2'})
        for expression in regexps:
            result = expression.search(line)
            if result is not None:
                if 'encryption' in result.groupdict() :
                    if result.groupdict()['encryption'] == 'on' :
                        cells[-1].update({'encryption': 'wep'})
                    else :
                        cells[-1].update({'encryption': 'off'})
                else :
                    cells[-1].update(result.groupdict())
                continue
    return cells



def send_mail(place, attachment=None):
    """
    Send mail with mailutils
    """
    sender_address = "unitreepro@jsk.imi.i.u-tokyo.ac.jp"
    receiver_address = "unitree@jsk.imi.i.u-tokyo.ac.jp"
    # receiver_address = "yanokura@jsk.imi.i.u-tokyo.ac.jp"
    mail_title = u"スパーキー、お散歩中です。"
    message = u"お散歩してます。\\n今は{}を歩いているよ。".format(place)
    cmd = u"echo -e '{}'".format(message)
    cmd += u" | /usr/bin/mail -s '{}' -r {} {}".format(
        mail_title, sender_address, receiver_address)
    if attachment is not None:
        cmd += ' -A {}'.format(attachment)
    exit_code = subprocess.call(cmd, shell=True)


class WalkNotifier(object):

    def __init__(self):
        # nmcli.disable_use_sudo()
        self.bridge = cv_bridge.CvBridge()
        self.img = None
        self.sub = rospy.Subscriber('/image_publisher/output',
                                    sensor_msgs.msg.Image,
                                    callback=self.callback,
                                    queue_size=1)
        rate = rospy.Rate(10)
        self.cnt = 0
        while not rospy.is_shutdown() and self.img is None:
            rate.sleep()
            rospy.loginfo('waiting image')

    def callback(self, img_msg):
        self.img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    def get_place(self):
        # aps = nmcli.device.wifi()
        # contents = [{"macAddress": ap.bssid, "age": 0} for ap in aps]
        aps = parse(scan('wlan1'))
        contents = [{"macAddress": str(ap['mac']), "age": 0} for ap in aps]
        headers = {
            'Content-type': 'application/json',
        }
        params = {
            'key': 'AIzaSyBGgqLPnccH8hWAFIIdtY68nv3Sf9uEFIA',
            'language': 'ja',
        }
        data = '{"wifiAccessPoints": ' + '[{}]'.format(contents) + '}'

        response = requests.post('https://www.googleapis.com/geolocation/v1/geolocate', params=params, headers=headers, data=data)
        # {'location': {'lat': 35.7145647, 'lng': 139.766433}, 'accuracy': 19.612}
        hoge = response.json()

        lat = hoge['location']['lat']
        lng = hoge['location']['lng']
        # lat = 35.715106109567415
        # lng = 139.77380123496505
        response = requests.get('https://maps.googleapis.com/maps/api/geocode/json?latlng={},{}'.format(lat, lng),
                                headers=headers, params=params)

        address = response.json()
        a = address['results'][0]['formatted_address']
        print_address = " ".join(a.split(' ')[1:])

        if self.img is not None:
            img_path = '/tmp/image-{0:04}.jpg'.format(self.cnt)
            cv2.imwrite(img_path, self.img)
            self.cnt += 1
            send_mail(print_address, img_path)
        else:
            send_mail(print_address)

    def run(self):
        seconds = 180
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                notifier.get_place()
            except Exception as e:
                print('error')
                print(e)
                continue
            print('send')
            print('waiting ...')
            time.sleep(seconds)


if __name__ == '__main__':
    rospy.init_node('walk_mail')
    notifier = WalkNotifier()
    notifier.run()
