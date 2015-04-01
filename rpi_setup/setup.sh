#!/bin/bash
##a script to set up the raspi
# y | sudo apt-get update
# y | sudo apt-get upgrade
# y | sudo apt-get install python-picamera python-serial git
# cd /tmp
# git clone https://github.com/lurch/rpi-serial-console.git
# cd rpi-serial*
#  sudo rpi-serial-console disable
# cd ..
# rm -rf rpi-serial*
# cd ~
##git clone https://github.com/ycoroneos/fishbrainz.git

# Configure Edimax wifi dongle
sudo apt-get install wpasupplicant
sudo cp wpa_supplicant.conf /etc/wpa_supplicant/
sudo cp 8192cu.conf /etc/modprobe.d/

# Configure fstab to mount mBed as usb drive
sudo mkdir /media/mbed
sudo sh -c "echo '/dev/sda     /media/mbed     vfat     defaults     0     2' >> /etc/fstab"

 
