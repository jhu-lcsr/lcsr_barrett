0.8.0 (2014-12-08)
------------------
* Adding inertia rate params
* updating mass rate params
* Contributors: Jonathan Bohren

0.7.0 (2014-12-08)
------------------
* removing unneeded dependencies
* Merge pull request `#16 <https://github.com/jhu-lcsr/lcsr_barrett/issues/16>`_ from nasnysom/cmake_depend
  attempting indigo build.
* moved cmake_modules dependency lines to catkin components list
* attempting indigo build. added cmake_modules packa ge to handle Eigen dependency
* Merge pull request `#15 <https://github.com/jhu-lcsr/lcsr_barrett/issues/15>`_ from svozar/wam40-cal
  Update WAM40 Calibration File
* Merge branch 'master' into wam40-cal
* Updated WAM 40 Calibration file
* adding additional error checking for startup
* Merge pull request `#13 <https://github.com/jhu-lcsr/lcsr_barrett/issues/13>`_ from svozar/upstream
  Adding WAM 40 calibration
* Merge branch 'master' of github.com:svozar/lcsr_barrett_nasa into upstream
* updating configuration for WAM 40
* fixing lua hw scripts
* Merge branch 'master' of github.com:jhu-lcsr/lcsr_barrett
* updating wam7 launchfile to use new calibration file
* Merge pull request `#12 <https://github.com/jhu-lcsr/lcsr_barrett/issues/12>`_ from svozar/patch-1
  Update README.md
* updating wam 60 calibration and creating calibration file for wm 40
* Update README.md
* adding moi rescaling flag to world sdf
* adding stand-alone wam60 calibration file
* resolving merge conflicts in hydra teleop
* Merge branches 'master' and 'nasa-lua'
* resolving merge of lua stuff
* Merge pull request `#1 <https://github.com/jhu-lcsr/lcsr_barrett/issues/1>`_ from jhu-lcsr/master
  Update from original
* Update README.md
* Merge pull request `#9 <https://github.com/jhu-lcsr/lcsr_barrett/issues/9>`_ from svozar/patch-5
  Update README.md
* Update README.md
* Merge pull request `#8 <https://github.com/jhu-lcsr/lcsr_barrett/issues/8>`_ from svozar/patch-4
  Update README.md
* Update README.md
* Merge pull request `#7 <https://github.com/jhu-lcsr/lcsr_barrett/issues/7>`_ from svozar/patch-3
  Update README.md
* Update README.md
* Merge pull request `#6 <https://github.com/jhu-lcsr/lcsr_barrett/issues/6>`_ from svozar/patch-2
  Update README.md
* Update README.md
* Merge pull request `#5 <https://github.com/jhu-lcsr/lcsr_barrett/issues/5>`_ from svozar/patch-1
  Update README.md
* Update README.md
* Merge pull request `#4 <https://github.com/jhu-lcsr/lcsr_barrett/issues/4>`_ from svozar/patch-1
  Update README.md
* Update README.md
* Update README.md
* Update README.md
* sim: adding args to launchfile for setting initial joint positions (note that gazebo still has issues with this sometimes)
* updating simulation parameters for gazebo 2.2.3, fixing some problems with ik controller params
* whitespace, reindenting, adding bhand mid-level control
* Merge branch 'lua-testing' of github.com:jhu-lcsr/lcsr_barrett into lua-testing
* sim: Adding verbose option and auto-enabling hand
* updating for ldeployer
* getting rid of ops scripts and cleaning up
* cleaning out old ops files, adding lua scripts
* Merge branch 'master' of github.com:jhu-lcsr/lcsr_barrett
* iters manip
* hydra: better gripper control
* updating 4dof wam
* Merge branch 'master' of github.com:jhu-lcsr/lcsr_barrett
* fixing empty-palm control parameters and adding aruco target urdf
* improving hydra mapping
* Merge branch 'master' of github.com:jhu-lcsr/lcsr_barrett
* updating hydra teleop with parameterizable command frame
* Contributors: John Choi, Jonathan Bohren, Steve Vozar, svozar

0.6.0 (2014-09-09)
------------------

* Tested cartesian impedance controller with WAM+BHand, controlled by Razer Hydra

