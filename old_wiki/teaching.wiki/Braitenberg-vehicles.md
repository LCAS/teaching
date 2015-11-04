# Creating the different types of Braitenberg vehicles

You have seen in the lecture how Braitenberg vehicles work, now it is your turn to make the turtlebot behave like one. First of all please make sure to follow the instructions on how to create a workspace and package if you haven't done so or if the one you have created has been removed from the PC. You can find the instruction on how to do this in [the First Turtlebot Coding tutorial](https://github.com/LCAS/teaching/wiki/First-Turtlebot-Coding#setting-up-your-local-ros-workspace).

After creating your workspace and package, please download the example file:

```
wget https://raw.githubusercontent.com/LCAS/teaching/indigo-devel/cmp3641m-code-fragments/scripts/braitenberg.py -O ~/catkin_ws/src/commanding_velocity/scripts/braitenberg.py
```

and make it executable

```
chmod a+x ~/catkin_ws/src/commanding_velocity/scripts/braitenberg.py
```

This file contains a working example of a Braitenberg vehicle type 1. This means the robot will drive forwards according to the intensity of the light in front of it. Since we do only have a kinect sensor, we are using this to emulate the light sensors on a real vehicle. Let's have a look at the code:

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                  # The ROS python bindings
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image


class Braitenberg():
    """A class to make a Braitenberg vehicle
    """

    # __init__ is a built-in python function and needs to start and end with *two* underscores
    def __init__(self, name):
        """Function to initialise the class. Called when creating a new instance

        :param name: The name of the ros node
        """

        rospy.loginfo("Starting node %s" % name)
        self.bridge = CvBridge()            # Creatingan OpenCV bridge object used to create an OpenCV image from the ROS image
        cv2.namedWindow("Image window", 1)  # Opening a window to show the image
        cv2.startWindowThread()

        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
            "/camera/rgb/image_color",      # The topic to which it should listened
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            "/cmd_vel",                     # The topic to which it should publish
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warning in ROS
        )

    def image_callback(self, img):
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to gray scale

        #######################################################################
        # CODE GOES HERE

        print "====="
        mean_intensity = numpy.mean(gray_image)           # Getting the mean intensity of the whole image
        normalised_mean_intensity = mean_intensity / 255  # Normalising the intensity
        print "Mean intensity: ", mean_intensity
        print "Normalised mean intensity: ", normalised_mean_intensity

        wheel_power = normalised_mean_intensity    # Using normalised intensity to determine the speed of the robot
        twist_msg = self.wheel_motor_power_to_twist_msg(wheel_power)  # Creating twist message from power value
        #######################################################################


        cv2.imshow("Image window", gray_image)  # Showing the image
        self.cmd_vel_pub.publish(twist_msg)     # Publishing the twist message


    # This function helps you to emulate e Braitenberg vehicle by allowing you
    # to virtually send velocities to the separate wheels. There is no need to
    # change this function at any point. Please refer to the example and the
    # task description on how to use it
    def wheel_motor_power_to_twist_msg(self, left_wheel_power, right_wheel_power=None):
        """Emulating a differential wheel drive where you can set the power for
        each wheel separately. Takes wheel powers between -1.0 and 1.0
        Used for Braitenberg vehicle type 2. If only left_wheel_power is set,
        the right wheel will have the same power to emulate a vehicle type 1.

        :param left_wheel_power: Power to the left wheel 1.0 fast forwards, -1.0 fast backwards
        :param right_wheel_power: Power to the right wheel 1.0 fast forwards, -1.0 fast backwards
        """

        # for Braitenberg vehicle type 1. Only setting left_wheel_power results
        # in both wheels receiving the same power
        if right_wheel_power == None:
            right_wheel_power = left_wheel_power

        # Making sure the power is between -1 and 1
        if left_wheel_power > 1.0 or left_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return
        if right_wheel_power > 1.0 or right_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return

        # Calculating linear and angular speed using fixed values. Results in a
        # top speed of up to 0.2 m/s
        left_wheel = [0.1*left_wheel_power, -1.0*left_wheel_power]
        right_wheel = [0.1*right_wheel_power, 1.0*right_wheel_power]

        # Generating and publishing the twist message
        twist = Twist()
        twist.linear.x = left_wheel[0] + right_wheel[0]
        twist.angular.z = left_wheel[1] + right_wheel[1]
        return twist


# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("braitenberg")     # Create a node of name braitenberg
    b = Braitenberg(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
```

We are using OpenCV for the image processing, numpy for the handling of the image matrices like in Matlab, `sensor_msgs/Image` as the built in ROS image type, and `geometry_msgs/Twist` to send commands to the robot:

```python
import rospy                                  # The ROS python bindings
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image
```

In the `__init__` function we crate a few helper variables to transform a ROS image into an OpenCV image and to display that image for convenience:

```python
self.bridge = CvBridge()            # Creatingan OpenCV bridge object used to create an OpenCV image from the ROS image
cv2.namedWindow("Image window", 1)  # Opening a window to show the image
cv2.startWindowThread()
```

We then create a publisher for our twist messages:

```python
self.cmd_vel_pub = rospy.Publisher( # The same as previously
    "/cmd_vel",                     # The topic to which it should publish
    Twist,                          # The data type of the topic
    queue_size=1                    # Explicitly set to prevent a warning in ROS
)
```

and a subscriber to trigger our callback every time the Kinect sends a new colour image:

```python
self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
    "/camera/rgb/image_color",      # The topic to which it should listened
    Image,                          # The data type of the topic
    callback=self.image_callback,   # The callback function that is triggered when a new message arrives
    queue_size=1                    # Disregard every message but the latest
)
```

We then have our callback (which is triggered when a new image arrives) that calculates the intensity of the image and controls the robots motors accordingly to make the robot drive forwards:

```python
    def image_callback(self, img):
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to gray scale

        #######################################################################
        # CODE GOES HERE

        print "====="
        mean_intensity = numpy.mean(gray_image)           # Getting the mean intensity of the whole image
        normalised_mean_intensity = mean_intensity / 255  # Normalising the intensity
        print "Mean intensity: ", mean_intensity
        print "Normalised mean intensity: ", normalised_mean_intensity

        wheel_power = normalised_mean_intensity    # Using normalised intensity to determine the speed of the robot
        twist_msg = self.wheel_motor_power_to_twist_msg(wheel_power)  # Creating twist message from power value
        #######################################################################


        cv2.imshow("Image window", gray_image)  # Showing the image
        self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message
```

The mean intensity is created from the mean of all the values of the gray scale image which is exactly what we have done in Computer Vision last year. This intensity, however, has to be normalised to create a value between 0 and 1 which is the input for a helper function called `wheel_motor_power_to_twist_msg`. This function returns a twist message generated from the motor power you set, which is then published to make it move. All this should be straight forward and the comments should give you a hint on what every line does. The helper function itself shouldn't be of great concern to you because it only emulates the ability of controlling the wheels directly which ROS otherwise doesn't provide.

## Tasks

Run spyder:

```
source ~/catkin_ws/devel/setup.bash
spyder
```

and open the file in `catkin_ws/src/command_velocities/scripts/braitenberg.py`


### Task 1: Creating a reverse Braitenberg vehicle

* Before altering the code, try to run it on the real robot, have a look at [how to start your turtlebot](https://github.com/LCAS/teaching/wiki/Turtlebots#using-your-turtlebot) and [how to connect to it](https://github.com/LCAS/teaching/wiki/Turtlebots#working-on-your-local-pc) if you don't remember how to. Observe the output of the programme and discuss why the robot behaves as it does.
* After you tried the code, alter the part:

 ```python
        #######################################################################
        # CODE GOES HERE

        print "====="
        mean_intensity = numpy.mean(gray_image)           # Getting the mean intensity of the whole image
        normalised_mean_intensity = mean_intensity / 255  # Normalising the intensity
        print "Mean intensity: ", mean_intensity
        print "Normalised mean intensity: ", normalised_mean_intensity

        wheel_power = normalised_mean_intensity    # Using normalised intensity to determine the speed of the robot
        twist_msg = self.wheel_motor_power_to_twist_msg(wheel_power)  # Creating twist message from power value
        #######################################################################
 ```
 to make the robot move away from a light source.
 * Try out this behaviour on the real robot

### Task 2: Creating a Braitenberg vehicle type 2

The very simple implementation of the type 1 vehicle only goes forwards (or backwards) with a speed depending on the intensity of the image. The type 2 vehicle has two sensors of which each one controls one of the wheels. We will try to emulate this now using only one Kinect.

* To emulate two sensors with only one image you have to split the image in half. In python, OpenCV represents images as matrices (exactly as in matlab). Try and recall how gray scale images (which is what we are working with) were represented in matlab.
* In python, OpenCV uses `numpy` to work on matrices. The matrices are accessed the same way as in matlab with the only difference that in python we start counting from 0: `gray_image[0,0]` therefore gives you the pixel in the top left corner of the image.
* To get the mean intensity of the left half of the image you have to call `numpy.mean()` only on a part of the image. If you don't remember how to index matrices in matlab, please have a look at the [basic indexing tutorial](http://docs.scipy.org/doc/numpy/user/basics.indexing.html) of numpy. Pay close attention to how to slice arrays and how to index multidimensional arrays.
* Don't forget to normalise the result as the helper function only takes values between -1 and 1
* Call the helper function to generate the twist message:

 ```python
twist_msg = self.wheel_motor_power_to_twist_msg(left_wheel_power, right_wheel_power)
 ```

**Task 2a:**
Make the robot drive away from a light source.

**Task 2b:** 
Make the robot drive towards a light source.

Use your mobile phone's flash light to influence the behaviour of the robot and discuss what you see on the image and the programmes output and what you can observe from the behaviour.

### Task 3: Create a ROS version of the Braitenberg vehicle

So far you used the mystery function that we gave you to translate the power to the separate wheels into a twist message, which is the normal way of controlling a robot in ROS. For this task you have to create this twist message yourself. Please do not try and copy the functionality in the helper function provided as this emulates the behaviour of a Braitenberg vehicle and is normally not the correct way of doing it for our robots.

* In order to be able to still use your phone's torch to influence the behaviour of the robot, it shouldn't go too fast. Make sure that the robot never goes faster than `0.3 m/s` and never turns faster than `PI/4 rad/s`.
* Use these maximum speeds and the mean intensities we calculated in all the previous tasks to create your own twist message. Do not use the `wheel_motor_power_to_twist_msg` any more!

**Task 3a:** Make the robot move away from light sources

**Task 3b:** Make the robot move towards light sources

Can you observe any differences in the behaviour compared to the Braitenberg vehicle? Is this kind of behaviour suitable for real world environments? What could this type of behaviour, also considering other stimuli than light, be useful for? 
