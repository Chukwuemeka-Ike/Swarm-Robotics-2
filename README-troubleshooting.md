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