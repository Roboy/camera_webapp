import rospy

from paramiko import SSHClient
from scp import SCPClient

import pyqrcode
from PIL import Image

import cups

PHONE_PATH='/photo/folder'
PKG_PATH=''
PHOTO_PATH=''

ssh_info = {
    'hostname': os.environ.get('SCP_HOSTNAME'),
    'port': int(os.environ.get('SCP_PORT')),
    'username': os.environ.get('SCP_USERNAME'),
}
ssh = SSHClient()
ssh.load_system_host_keys()
ssh.connect(**ssh_info)

do_scp = True
generate_qr = True
watermark = True

def generate_qr_png(url, logo='RoboyLogoCut.png', name='qr.png'):
    url = pyqrcode.QRCode(url,error = 'H')
    url.png('test.png',scale=15)
    im = Image.open('test.png')
    im = im.convert("RGB")
    logo = Image.open(logo)
    x = im.width/2 - im.width/6
    y = im.width/2 + im.width/6
    box = (x,x,y,y)
    im.crop(box)
    region = logo
    region = region.resize((box[2] - box[0], box[3] - box[1]))
    im.paste(region,box, mask=region)
    im.save(name, "PNG")

def print_photo(path, printer='Canon_CP910'):
    conn = cups.Connection()
    conn.printFile(printer, path, "", {})


def watermark_with_transparency(cv_image,
                                logo,
                                position=(0,0)):
    base_image = Image.fromarray(cv_image)
    watermark = Image.open(logo)
    watermark = watermark.resize((1080,720))
    width, height = base_image.size

    transparent = Image.new('RGB', (width, height), (0,0,0,0))
    transparent.paste(base_image, (0,0))
    transparent.paste(watermark, position, mask=watermark)
    open_cv_image = np.array(transparent)
    # Convert RGB to BGR
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    return open_cv_image

def cb(msg):
	filename = msg.data
	image = cv2.imread(PHONE_PATH+filename)

	if watermark:
		image = watermark_with_transparency(cv_image=image, logo=PKG_PATH+'/img/lower_border_v0.png')
	
	cv2.imwrite(PHOTO_PATH+filename, image)
	rospy.set_param('snapchat/latest_filename', filename.split('.')[0])

	if do_scp:
        qr = generate_qr_png(url='https://bot.roboy.org/%s'%filename, name='qr_%s.png'%filename.split('.')[0], logo=path+'/img/logo.png')

        with SCPClient(ssh.get_transport()) as scp:
            scp.put(filename, '/var/www/html')
            scp.put('qr_%s.png'%filename.split('.')[0], '/var/www/html')



def main():
	rospy.init_node("photo_server")
	rospy.Subscriber("/latest_photo", String, callback)
	rospy.Subscriber("/print_photo", String, print_cb)
	rospy.spin()
