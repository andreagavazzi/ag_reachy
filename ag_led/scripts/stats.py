#!/usr/bin/env python

import rospy
#from std_msgs.msg import String

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess


def stat():
    
    # pub_l1 = rospy.Publisher('lcd_line', String, queue_size=10)
    rospy.init_node('led', anonymous=False)
    rospy.loginfo('Node Started.')
    rate = rospy.Rate(1)

    # 128x32 display with hardware I2C:
    disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1) # gpio to 1 avoid platform detection
    # Initialize library.
    disp.begin()
    # Clear display.
    disp.clear()
    disp.display()


    # Create blank image for drawing.
    # Make sure to create image with mode '1' for 1-bit color.
    width = disp.width
    height = disp.height
    image = Image.new('1', (width, height))

    # Get drawing object to draw on image.
    draw = ImageDraw.Draw(image)

    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)

    # Draw some shapes.
    # First define some constants to allow easy resizing of shapes.
    padding = -2
    top = padding
    bottom = height-padding
    # Move left to right keeping track of the current x position for drawing shapes.
    x = 0

    # Load default font.
    font = ImageFont.load_default()
    
    while not rospy.is_shutdown():
        # Draw a black filled box to clear the image.
        draw.rectangle((0,0,width,height), outline=0, fill=0)

        # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        try:
            cmd = "hostname -I | cut -d\' \' -f1"
            IP = subprocess.check_output(cmd, shell = True )
            cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
            CPU = subprocess.check_output(cmd, shell = True )
            cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
            MemUsage = subprocess.check_output(cmd, shell = True )
            cmd = "free -m | awk 'NR==3{printf \"Swap: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
            SwapUsage = subprocess.check_output(cmd, shell = True )
            cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
            Disk = subprocess.check_output(cmd, shell = True )
            cmd = "date"
            Date = subprocess.check_output(cmd, shell = True )
            Text = "Reachy by agRobotics"
        except:
            pass

        # Write lines of text.
        draw.text((x, top),       str(Text.decode('utf-8')),  font=font, fill=255)
        draw.text((x, top+16),    "IP: " + str(IP.decode('utf-8')),  font=font, fill=255)
        draw.text((x, top+24),    str(CPU.decode('utf-8')), font=font, fill=255)
        draw.text((x, top+32),    str(MemUsage.decode('utf-8')),  font=font, fill=255)
        draw.text((x, top+40),    str(SwapUsage.decode('utf-8')),  font=font, fill=255)
        draw.text((x, top+48),    str(Disk.decode('utf-8')),  font=font, fill=255)


        # Display image.
        disp.image(image)
        disp.display()
        rospy.sleep(1.0)
    
    disp.clear()
    disp.display()
    rospy.loginfo('Node terminated.')
    

# Main routine
if __name__ == '__main__':
    try:
        stat()
    except rospy.ROSInterruptException:
        disp.clear()
        disp.display()
        pass