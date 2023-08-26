## Error in REST request
You are openning Gazebo the robot simulator on Ubuntu 18.04 Bionic, ROS Melodic.

But you see this problem in red in the terminal:

```
[Err] [REST.cc:205] Error in REST request

libcurl: (51) SSL: no alternative certificate subject name matches target host name ‘api.ignitionfuel.org’
```

Open `~/.ignition/fuel/config.yaml`, replace
```
api.ignitionfuel.org
```
with
```
fuel.ignitionrobotics.org
```

## Gazebo does not shutdown properly when you exit?
Try couple of times:
``` bash
killall gzclient && killall gzserver && killall rosmaster
```
You can also add it to your `~/.bashrc` as an alias named `killg`:
``` bash
# TO KILL GAZEBO CLIENT AND SERVER:
alias killg='killall gzclient && killall gzserver && killall rosmaster'
```


## Error Building UWB Gazebo Plugin
If ```catkin_make``` cannot build the UWB Gazebo plugin, with beginning of the error similar to the snippet below, there may be an issue with the CMake version.
```bash
[ 97%] Building CXX object uwb_gazebo_plugin/CMakeFiles/uwb_plugin.dir/src/UwbPlugin.cpp.o
In file included from /usr/include/ignition/math6/gz/math/Quaternion.hh:20,
                 from /usr/include/ignition/math6/gz/math/Pose3.hh:20,
                 from /usr/include/ignition/math6/ignition/math/Pose3.hh:18,
                 from /usr/include/sdformat-9.8/sdf/Actor.hh:23,
                 from /usr/include/sdformat-9.8/sdf/sdf.hh:2,
                 from /usr/include/gazebo-11/gazebo/common/Plugin.hh:33,
                 from /home/mekahertz/catkin_ws_swarm2/src/uwb_gazebo_plugin/src/UwbPlugin.cpp:24:
/usr/include/ignition/math6/gz/math/Helpers.hh: In function ‘std::chrono::_V2::steady_clock::duration ignition::math::v6::stringToDuration(const string&)’:
/usr/include/ignition/math6/gz/math/Helpers.hh:1059:28: error: ‘chrono_literals’ is not a namespace-name
 1059 |       using namespace std::chrono_literals;
      |                            ^~~~~~~~~~~~~~~
```
This project has only been successfully built using CMake 3.16.3. To check the CMake version, run
```bash
cmake --version
```
If the version is different from 3.16.3, catkin might be using that version and causing the error above. Suggestion would be to remove CMake and install the needed version. Otherwise, you will have to reorganize your PATH to ensure it uses the correct one.