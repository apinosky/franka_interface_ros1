# bash file to connect to robot

CATKIN_SHELL=bash
source $HOME/franka_ws/devel/setup.bash

extra=${1-0}

## camera settings (moved to roslaunch file)
# v4l2-ctl -c white_balance_temperature_auto=1 # to turn off auto
#v4l2-ctl -c white_balance_temperature_auto=0 # to turn off auto
#v4l2-ctl -c white_balance_temperature=8000
#v4l2-ctl -c white_balance_temperature=3000
# v4l2-ctl -c exposure_auto=1 # set to manual
#v4l2-ctl -c exposure_absolute=87

## torque sensor settings
if [ $extra == gs ]
then
  expect ati_telnet.expect 2.75 # gelsight
elif [ $extra == cam ]
then
  expect ati_telnet.expect 3.75 # camera
fi
