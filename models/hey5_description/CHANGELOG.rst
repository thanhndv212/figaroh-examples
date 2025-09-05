^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hey5_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.4 (2023-11-14)
------------------
* Add website tag
* Contributors: Noel Jimenez

3.0.3 (2023-06-27)
------------------
* create hey5 ros2_control and transmission xacro files
* Contributors: Noel Jimenez

3.0.2 (2022-11-29)
------------------
* Merge branch 'rename_namespace' into 'humble-devel'
  rename namespace for gazebo plugin
  See merge request robots/hey5_description!9
* rename namespace for gazebo plugin
* Contributors: Jordan Palacios, Noel Jimenez

3.0.1 (2022-11-03)
------------------
* Merge branch 'fix_dependency' into 'humble-devel'
  fix buildtool dependency
  See merge request robots/hey5_description!8
* fix buildtool dependency
* Contributors: Jordan Palacios, Noel Jimenez

3.0.0 (2022-10-26)
------------------
* Merge branch 'linters' into 'humble-devel'
  add linters
  See merge request robots/hey5_description!7
* CONTRIBUTING.md
* add linters
* Merge branch 'update_copyright' into 'humble-devel'
  update copyright and license
  See merge request robots/hey5_description!6
* restore original LICENSE
* rename LICENSE
* update copyright and license
* Merge branch 'update_maintainers' into 'humble-devel'
  update maintainers
  See merge request robots/hey5_description!5
* update maintainers
* Merge branch 'migrate_gazebo_plugin_underactuated_finger' into 'foxy-devel'
  Migrate gazebo plugin underactuated finger
  See merge request robots/hey5_description!4
* change pid gains values
* add pid_gains to urdf
* Remove comments to workaround https://github.com/ros2/launch_ros/issues/214
* package.xml and CMakeLists.txt in ros2 format
* Contributors: Jordan Palacios, Noel Jimenez, cescfolch, victor

1.0.3 (2019-04-15)
------------------
* Fix xacro warnings
* Contributors: Victor Lopez

1.0.2 (2018-04-13)
------------------
* Merge branch 'add-tool-link' into 'master'
  Add hand_tool_link
  See merge request robots/hey5_description!3
* Add hand_tool_link
* Contributors: Hilario Tome, Victor Lopez

1.0.1 (2018-01-15)
------------------
* Merge branch 'fix-reemc' into 'master'
  Fix hey5 hand for REEM-C robot
  See merge request !2
* Fix hay5 hand for REEM-C robot
* Merge branch 'multiple-tiagos' into 'master'
  Allow multiple Tiagos on Gazebo
  See merge request !1
* Allow multiple Tiagos on Gazebo
  Refs #15402
* Contributors: David Fernandez, Hilario Tome, Victor Lopez, davidfernandez

1.0.0 (2016-09-01)
------------------

0.1.3 (2016-04-25)
------------------
* Fix error on loading stil by Rviz as they seem to be malformed
  Error was
  The STL file 'package://XXXX/meshes/finger_flex_collision.stl' is malformed. According to the binary STL header it should have '108' triangles, but it has too much data for that to be the case.
  Seems to be fixed in upcoming Rviz (not released yet) https://github.com/ros-visualization/rviz/issues/913
* Install LICENSE.txt
* Change URDF license headers to CC BY 4.0.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Sam Pfeiffer

0.1.2 (2015-06-12)
------------------
* hey5_description: Install gazebo folder
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.1.1 (2015-06-12)
------------------
* Add hey5 gazebo simulation plugin
* Add gazebo references with black material
* Contributors: Bence Magyar, Luca Marchionni

0.1.0 (2015-04-22)
------------------
* Add hand_grasping_frame
* Contributors: Bence Magyar

0.0.6 (2015-01-15)
------------------
* Update hand inertial properties
  - Inertia tensors for all links except palm are approximately four
  orders of magnitude larger to prevent the Gazebo simulation from
  becoming unstable.
  - Palm mass is lower to take into account that actuator mass is
  specified separately.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.5 (2014-12-23)
------------------
* License model as CC-BY 4.0, add attribution text
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.4 (2014-12-12)
------------------
* Remove hi-res finger meshes
* Minor dimension tweaks to harmonize with docs, CAD
* Update hand transmissions, fully open config
  - Add some overlap between abduction and flexion to more closely mimic
  hardware
  - Spread fingers more on fully open configuration
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.3 (2014-11-13)
------------------
* Remove reference to non-existing file
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.2 (2014-11-13)
------------------
* Use transmissions in hand model
* Remove usage of mimic joints
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.1 (2014-11-05)
------------------
* First version of the Hey5 hand model
* Contributors: Adolfo Rodriguez Tsouroukdissian
