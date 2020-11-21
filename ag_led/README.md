# SOFTWARE INSTALLATION
For the OLED display to work the software from Adafruit is required. You can download them from GitHub with the following command.

```
git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
```

Then you change to the folder Adafruit_Python_SSD1306.

```
cd Adafruit_Python_SSD1306
```

Now please execute the following command for the installation.

```
sudo python setup.py install
```

```
sudo apt-get install libjpeg-dev zlib1g-dev
```

# CONFIGURING THE I²C BUS
After all programs and libraries are installed, the I²C must be configured. The user e.g. nano with whom you log on to your Jetson Nano must still receive group authorisation to access the I² bus. This is exactly what the following command does to adjust the group. For <username> please use your user with which you register.

```
sudo usermod -aG i2c <username>
```

Now please restart the Jetson Nano with the following command. After reboot the user you used can access the I²C bus and see the connected devices like the OLED display

```
sudo reboot
```

After the restart execute the following command and you should see the OLED display with the standard I²C address.

```
sudo i2cdetect -y -r 1
```