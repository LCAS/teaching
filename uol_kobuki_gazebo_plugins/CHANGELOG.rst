^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uol_kobuki_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2015-01-15)
------------------
* Adding the possibility of teleoperating the turtlebots via key op, see tutorial.md. Changing topic names to have the command velocities published under . Using the yocs_command_velocity_smoother as suggested by kobuki.
* Contributors: Christian Dondrup

0.1.3 (2015-01-14)
------------------
* set version correctly
* Now the modified kobuki node is only used in the multirobot launch file. The standard examples still use the unaltered one. Fixing `#3 <https://github.com/LCAS/teaching/issues/3>`_.
* Contributors: Christian Dondrup, Marc Hanheide
