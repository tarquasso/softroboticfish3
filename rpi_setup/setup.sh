#!/bin/bash
##a script to set up the raspi

# sudo apt-get install python-picamera python-serial git
#git clone https://github.com/lurch/rpi-serial-console.git
#cd rpi-serial*
#sudo rpi-serial-console disable
#cd ..
#rm -rf rpi-serial*
#cd ~
##git clone https://github.com/ycoroneos/fishbrainz.git


# ASSUMES partition table is already prepared

## Configure Edimax wifi dongle
sudo apt-get install wpasupplicant
sudo cp wpa_supplicant.conf /etc/wpa_supplicant/
sudo cp 8192cu.conf /etc/modprobe.d/
sudo cp fstab	/etc/fstab