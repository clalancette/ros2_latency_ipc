# ros2_latency_ipc

## What ?
Two sample ros nodes to measure latency for a single publisher / subscribe connection for interprocess communication.

## How to build ?
1. source your ROS2 distro
2. ```colcon build```

## How to run ?

To run the nodes with the out-of-the-box config, do the following:

### Terminal 1

This will wait for data to come over the topic.
Once as much data as possible is received, it will print a report outlining how many messages were received, their latency for delivery, and other statistics.
Note that if this never returns, it means that the last packet from the sender was probably dropped, and hence too many messages are being dropped.

```
ros2 run latency_rec latency_rec -d 0
```

### Terminal 2

This will send 100 packets (`-r 100`), plus 10 warmup packets, at 1ms intervals (`-d 1`), with each packet being of size 8MB (`-s 8192`):

```
ros2 run latency_snd latency_snd -r 100 -s 8192 -d 1
```

## How to run with Fast-DDS tuning ?

To run the nodes with a shared-memory only Fast-DDS tuning for sizes up to 8192, do the following:

### Terminal 1

```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=src/ros2_latency_ipc/fastdds-shm-only-config.xml RMW_FASTRTPS_USE_QOS_FROM_XML=1 ros2 run latency_rec latency_rec -d 0
```

### Terminal 2

```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=src/ros2_latency_ipc/fastdds-shm-only-config.xml RMW_FASTRTPS_USE_QOS_FROM_XML=1 ros2 run latency_snd latency_snd -r 100 -s 8192 -d 1
```
