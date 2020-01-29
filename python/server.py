#!/usr/bin/env python3
import os

import rospy
import rospkg
from std_msgs.msg import String

import cv2
import numpy as np
from paramiko import SSHClient
from scp import SCPClient

import pyqrcode
from PIL import Image

import cups

from adbsync import sync

PHONE_PATH='/home/roboy/Pictures/pixel/Download/'
PKG_PATH=os.path.dirname(os.path.realpath(__file__))+'/../'
PHOTO_PATH=PKG_PATH+'mwc/'

do_scp = False
generate_qr = True
watermark = True

if do_scp:
    ssh_info = {
        'hostname': os.environ.get('SCP_HOSTNAME'),
        'port': int(os.environ.get('SCP_PORT')),
        'username': os.environ.get('SCP_USERNAME'),
    }
    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.connect(**ssh_info)



def generate_qr_png(url, logo='RoboyLogoCut.png', name='qr.png', destination=''):
    url = pyqrcode.QRCode(url,error = 'H')
    url.png('test.png',scale=15)
    im = Image.open('test.png')
    im = im.convert("RGB")
    logo = Image.open(logo)
    x = int(im.width/2 - im.width/6)
    y = int(im.width/2 + im.width/6)
    box = (x,x,y,y)
    im.crop(box)
    region = logo
    region = region.resize((box[2] - box[0], box[3] - box[1]))
    im.paste(region,box, mask=region)
    im.save(destination+name, "PNG")

def print_photo(path, printer='DP-DS620'):
    conn = cups.Connection()
    conn.printFile(printer, path, "", {})


def watermark_with_transparency(cv_image,
                                logo,
                                position=(0,0)):
    cv_image = cv_image[:, :, ::-1].copy()
    base_image = Image.fromarray(cv_image)
    watermark = Image.open(logo)
    watermark = watermark.resize((960,720))
    # watermark = watermark.convert('RGB')
    width, height = base_image.size

    transparent = Image.new('RGB', (width, height), (0,0,0,0))
    transparent.paste(base_image, (0,0))
    transparent.paste(watermark, position, mask=watermark)
    open_cv_image = np.array(transparent)
    # Convert RGB to BGR
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    return open_cv_image

def cb(msg):
    sync() # copy new Android files 
    filename = msg.data
    image = cv2.imread(PHONE_PATH+filename)

    if watermark:
        image = watermark_with_transparency(cv_image=image, logo=PKG_PATH+'/img/lower_border_v0.png', position=(0,480))
    
    cv2.imwrite(PHOTO_PATH+filename, image)
    rospy.set_param('snapchat/latest_filename', filename.split('.')[0])
    rospy.loginfo("Saved %s"%filename)
    if do_scp:
        if generate_qr:
            qr = generate_qr_png(url='https://bot.roboy.org/%s'%filename, name='qr_%s.png'%filename.split('.')[0], logo=PKG_PATH+'/img/logo.png', destination=PHOTO_PATH)

        with SCPClient(ssh.get_transport()) as scp:
            scp.put(PHOTO_PATH+filename, '/var/www/html')
            if generate_qr:
                scp.put(PHOTO_PATH+'qr_%s.png'%filename.split('.')[0], '/var/www/html')

def print_cb(msg):
    filename = msg.data.strip(' ')
    path = PHOTO_PATH+filename+'.jpeg'
    rospy.loginfo("printing %s"%path)
    print_photo(path)

def main():
    rospy.init_node("photo_server")
    rospy.Subscriber("/latest_photo", String, cb)
    rospy.Subscriber("/print_photo", String, print_cb)
    rospy.loginfo("Photo server is up")
    rospy.spin()

if __name__ == '__main__':
  main()