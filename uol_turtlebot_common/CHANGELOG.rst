^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uol_turtlebot_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.16 (2017-03-29)
-------------------

0.1.15 (2017-02-23)
-------------------

0.1.14 (2017-02-21)
-------------------

0.1.13 (2015-11-19)
-------------------

0.1.12 (2015-04-22)
-------------------
* A launch file also starting the navigation and reducing the CPU load by not registering the depth and rgb image.
* Contributors: Christian Dondrup

0.1.11 (2015-04-20)
-------------------
* Adding necessary build depends.
* Offering service to reenable. Blinking leds on back yellow and red to visualise a collision.
* listening to bumper and sending stop continuously.
* Contributors: Christian Dondrup

0.1.10 (2015-02-20)
-------------------

0.1.9 (2015-02-03)
------------------

0.1.8 (2015-02-02)
------------------
* Added custom launch file for all of the turtlebot functionalities and a cmd_vel republisher to uol_turtlebot_common
* Contributors: Christian Dondrup

0.1.7 (2015-01-23)
------------------
* A bit too much copy and paste
* Reviving the uol_turtlebot_common package
  Installing all the necessary packages to run the actual robot. This makes the custom turtlebot workspace obsolete.
* Contributors: Christian Dondrup

0.1.6 (2015-01-21)
------------------

0.1.5 (2015-01-20)
------------------

0.1.4 (2015-01-15)
------------------
* Remote uol_turtlebot_common package. Removed dependencies from uol_morse_simulator package that do not exist in hydro yet.
* turtlebot_core_apps does not exist in indigo. turtlebot_apps is a metapackage and has therefore been removed from the cmake file.
* removed
* Contributors: Christian Dondrup, Marc Hanheide

0.1.3 (2015-01-14)
------------------
* new changelogs
* First version of simulation with one turtlebot
* First tutorial version only containing an empty world and keyboard teleop.
* Contributors: Christian Dondrup, Marc Hanheide, cdondrup

0.1.2 (2015-01-09 12:19)
------------------------
* updated
* Contributors: Marc Hanheide

0.1.1 (2015-01-09 11:47)
------------------------
* added changelogs
* bumped version after moving packages
* initialised with basic packages
* Contributors: Marc Hanheide
