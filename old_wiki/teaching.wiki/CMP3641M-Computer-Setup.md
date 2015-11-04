# Setting up your computer

The whole software infrastructure is based on a standard [ROS Indigo Installation](http://wiki.ros.org/indigo/Installation/Ubuntu) running on a 64-bit Installation of [Ubuntu 14.04LTS](http://www.ubuntu.com/download/desktop). 

All required software is available on the School's lab PCs when booted into Ubuntu 14.04. To install the required software packages on a fresh installation follow these simple steps (you need to open a "terminal" to enter the required commands, you may want to read [this tutorial](http://www.howtogeek.com/140679/beginner-geek-how-to-start-using-the-linux-terminal/) if you have never used a Linux terminal before):

1. Install Ubuntu 14.04 (if not already done) as detailed [here](http://www.ubuntu.com/download/desktop) (Make sure you choose 14.04 64bit)
1. Install ROS "Indigo" as described [here](http://wiki.ros.org/indigo/Installation/Ubuntu) (make sure you choose "ROS Indigo for Ubuntu 14.04"). Choose to install ros-indigo-desktop-full by entering `sudo apt-get install ros-indigo-desktop-full` in your terminal
1. Enable the LCAS package repositories:
  1. Add the LCAS public key to verify packages: 

     ```curl -s http://lcas.lincoln.ac.uk/repos/public.key | sudo apt-key add -```
  1. Add the LCAS repository: 

     ```sudo apt-add-repository http://lcas.lincoln.ac.uk/repos/release```
  1. update your index: 

     ```sudo apt-get update```
1. Install the package "uol_cmp3641m" which will install all required packages: 

  ```sudo apt-get install ros-indigo-uol-cmp3641m```