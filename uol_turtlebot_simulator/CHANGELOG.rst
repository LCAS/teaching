^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uol_turtlebot_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-03-15)
------------------
* configure fov and no range restriction (`#37 <https://github.com/LCAS/teaching/issues/37>`_)
  * configure fov and no range restriction
  * changed close cut-off to not hit base
  * wip
  * scan range
* Contributors: Marc Hanheide

1.0.4 (2020-02-25)
------------------
* Adding 3 mazes for testing the assignment (`#36 <https://github.com/LCAS/teaching/issues/36>`_)
  * WIP: new mazes and example solution
  * not here
  * 3 mazes and lanuch files
  * fixed the models (removed state)
  * fixing position of blue markers
  Co-authored-by: Marc Hanheide <marc@hanheide.net>
* new color marker
* added missing dep
* added missing dep
* added missing dep
* Contributors: Marc Hanheide, francescodelduchetto

1.0.3 (2020-01-29)
------------------
* updated gzmaze
* Contributors: Marc Hanheide

1.0.2 (2020-01-29)
------------------
* gzmaze fixes and better default camera and light
* Contributors: Marc Hanheide

1.0.1 (2020-01-28)
------------------
* update gzmaze
* Contributors: Marc Hanheide

1.0.0 (2020-01-28)
------------------
* working towards new assessment with maze (`#34 <https://github.com/LCAS/teaching/issues/34>`_)
  * added gzmaze editor
  * updated gzmaze
  * added build_depends
  * simple lanuch and maze world
  * changes for week 1 tutoral
  * correct collisions
* Merge pull request `#33 <https://github.com/LCAS/teaching/issues/33>`_ from paul-baxter/kinetic
* minor position change for cylinder A (red)
* change target positions for 18-19 assessment
* removed outdated materials
* Contributors: Marc Hanheide, Paul Baxter, paul-baxter

0.2.0 (2019-01-11)
------------------
* 0.1.19
* changelogs
* changelogs
* added rviz config for search worlds
* on way to working kinetic
* change position of test arena target poles for assessment
* Contributors: Marc Hanheide, paul-baxter

0.1.18 (2017-06-26 12:46)
-------------------------
* updated changelogs
* changed to new kinetic launch style
* Contributors: LCAS build farm, Marc Hanheide

0.1.17 (2017-06-26 09:58)
-------------------------
* changelogs
* removed packages that where invalid for kinetic
* Contributors: Marc Hanheide

0.1.16 (2017-03-29)
-------------------
* updated changelogs
* real test world for assessment
* Contributors: Jenkins, Marc Hanheide

0.1.15 (2017-02-23)
-------------------
* updated changelogs
* Merge pull request `#17 <https://github.com/LCAS/teaching/issues/17>`_ from LCAS/assessment
  * added assessment launch and world files
  * enabled gmapping, with somewhat working gmapping params
  * temporary slam maps
  * updated world from slack
  * first slam map okay-ish
* renabled publish tf
* removed turtlebot from world
* first slam map okay-ish
* updated world from slack
* temporary slam maps
* somewhat working gmapping params
* enabled gmapping, but not working well yet
* added assessment launch and world files
* Contributors: Jenkins, Marc Hanheide

0.1.14 (2017-02-21)
-------------------
* updated changelogs
* Merge branch 'indigo-devel' of https://github.com/LCAS/teaching into indigo-devel
* Update tutorial.md
* Update tutorial.md
* Update tutorial.md
* Update tutorial.md
* Update tutorial.md
* Update tutorial.md
* Update tutorial.md
* Merge branch 'indigo-devel' of https://github.com/LCAS/teaching into indigo-devel
  Conflicts:
  cmp3641m-code-fragments/scripts/color_contours.py
  cmp3641m-code-fragments/scripts/opencv_bridge.py
  cmp3641m-code-fragments/scripts/opencv_intro.py
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

0.1.13 (2015-11-19)
-------------------
* updated changelogs
* Merge pull request `#16 <https://github.com/LCAS/teaching/issues/16>`_ from cdondrup/indigo-devel
  Splitting labc.launch into two launch files
* Splitting labc.launch into two launch files and have lbac.launch as the meta launch file. This way it is easier to avoid the race condition on the lab machines.
* Merge branch 'indigo-devel' of https://github.com/LCAS/teaching into indigo-devel
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

0.1.12 (2015-04-22)
-------------------
* updated changelogs
* Contributors: Jenkins

0.1.11 (2015-04-20)
-------------------
* updated changelogs
* Contributors: Jenkins

0.1.10 (2015-02-20)
-------------------
* updated changelogs
* Merge pull request `#12 <https://github.com/LCAS/teaching/issues/12>`_ from LCAS/marc-hanheide-new-box-size
  change box size to the real one
* change box size to the real one
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

0.1.9 (2015-02-03)
------------------
* updated changelogs
* Merge pull request `#10 <https://github.com/LCAS/teaching/issues/10>`_ from cdondrup/indigo-devel
  Adding a green box on top of each robot.
* Adding a green box on top of each robot.
* Contributors: Christian Dondrup, Jenkins

0.1.8 (2015-02-02)
------------------
* Updated changelogs
* Contributors: Christian Dondrup

0.1.7 (2015-01-23)
------------------

0.1.6 (2015-01-21)
------------------
* updated changelogs
* Contributors: Jenkins

0.1.5 (2015-01-20)
------------------
* updated changelogs
* Adding the essential uol_kobiku node to the package.xml
* Contributors: Christian Dondrup, Marc Hanheide

0.1.4 (2015-01-15)
------------------
* changelogs
* Merge branch 'indigo-devel' of https://github.com/cdondrup/teaching into cdondrup-indigo-devel
* Updated tutorial for indigo.
* Merging hydro-devel
* Merge branch 'hydro-devel' of https://github.com/LCAS/teaching into hydro-devel
* Adding the possibility of teleoperating the turtlebots via key op, see tutorial.md. Changing topic names to have the command velocities published under . Using the yocs_command_velocity_smoother as suggested by kobuki.
* Contributors: Christian Dondrup, Marc Hanheide

0.1.3 (2015-01-14)
------------------
* new changelogs
* Merge pull request `#4 <https://github.com/LCAS/teaching/issues/4>`_ from cdondrup/hydro-devel
  Adding simulation of 2 turtlebots in Comp Lab C including short tutorial
* Minor changes made during meeting.
* Create correct install targets and removed unnecessary launch files.
* Now the modified kobuki node is only used in the multirobot launch file. The standard examples still use the unaltered one. Fixing `#3 <https://github.com/LCAS/teaching/issues/3>`_.
* Now works with two robots but not one anymore.
* First version of simulation with one turtlebot
* Contributors: Christian Dondrup, Marc Hanheide

0.1.2 (2015-01-09 12:19)
------------------------

0.1.1 (2015-01-09 11:47)
------------------------
