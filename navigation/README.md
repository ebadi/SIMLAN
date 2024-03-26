## Build instruction

First make sure that the dependencies and packages on the parent [README.md](../README.md) are installed.  

Start by building all ROS2 packages:

```bash
./build.sh
```

## Manual control and data collection
To control the infobot using keyboard:

```bash
./teleop.sh
```

(optionally) To record ros messages in ROS bag files:

```bash
./record.sh
```

To replay the last rosbag recording:

```bash
./replay-last.sh
```

```
Files:             rosbag2_2024_03_18-07_53_18_0.db3
Bag size:          77.0 KiB
Storage id:        sqlite3
Duration:          30.980s
Start:             Mar 18 2024 07:53:21.942 (1710748401.942)
End:               Mar 18 2024 07:53:52.923 (1710748432.923)
Messages:          554
Topic information: Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 554 | Serialization Format: cdr

[INFO] [1710775265.896618797] [rosbag2_storage]: Opened database 'rosbag2_2024_03_18-15_16_56/rosbag2_2024_03_18-15_16_56_0.db3' for READ_ONLY.
```

(optionally) To do cartography: 

```
./cartographer.sh
```


## Autonomous control (using nav2)
To start nav2 navigation stack and start commanding the infobot to move in the map:
:

```bash
./navigation.sh
```
and then: 

```bash
./commander.sh
```
