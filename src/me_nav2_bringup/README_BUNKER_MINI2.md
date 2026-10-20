# BUNKER MINI 2.0 Integration

The base driver consumes Nav2 commands on `/cmd_vel`, publishes wheel odometry
on `/wheel_odom`, and publishes status on `/bunker_status` and
`/bunker_rc_state`.

The navigation stack's LIO pipeline remains the only publisher of
`odom -> base_footprint`. The base driver's TF publisher is disabled by the
adapter launch file to avoid conflicting transforms.

## Build

Install the SDK dependency once, then build the base packages and the adapter:

```bash
sudo apt-get install libasio-dev can-utils
cd /home/demo/Lidar_nav2_ws-main/Lidar_nav2_ws
colcon build --packages-select ugv_sdk bunker_msgs bunker_base me_nav2_bringup --symlink-install
source install/setup.bash
```

## Bring up the CAN interface

The BUNKER MINI 2.0 driver uses SocketCAN at 500 kbit/s. Bring-up is scripted in
`scripts/chassis_start.sh`, which loads `gs_usb`, brings the interface up, and
samples the bus with `candump` to confirm frames are flowing:

```bash
./scripts/chassis_start.sh --can-only   # configure the interface only
./scripts/chassis_start.sh              # configure it, then start the driver
```

To do it by hand instead, with the AgileX CAN-to-USB adapter connected and the
vehicle powered on:

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
candump can0
```

Confirm that `candump` receives frames before starting ROS. Keep the remote
controller ready and make sure the emergency stop is released.

Use the `CAN_IFACE` environment variable (`CAN_IFACE=can1`) when the adapter is
assigned another SocketCAN name, and `CAN_BITRATE` if the bus is not 500 kbit/s.

## Start the base adapter

`scripts/nav2_real.sh` owns chassis bring-up: it runs the CAN configuration in the
foreground first, so the `sudo` prompt can be answered in that terminal, and only
continues with the navigation stack if that succeeds. It then opens a "BUNKER
底盘驱动" terminal running `scripts/chassis_start.sh` before bringing up the
rest of the stack.

Do **not** start the adapter by hand beforehand. `nav2_real.sh` terminates
leftover `ros2 launch me_nav2_bringup` processes on entry, which includes the base
driver — anything you started manually gets killed and restarted.

To run the driver on its own, for example on a bench without the navigation stack:

```bash
./scripts/chassis_start.sh
```

Only ever run a single base driver instance: two of them both consume `/cmd_vel`
and both write CAN frames to the vehicle.

The existing robot description must contain the measured fixed transform from
`base_footprint` to `livox_frame`. Update
`gld_robot_description/model/gld_robot_description.urdf` with the actual
sensor installation offsets before using localization or autonomous motion.
