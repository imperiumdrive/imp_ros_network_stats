## **What Is This?**

This package contains the following ROS 2 nodes:

1. `listener`
2. `talker`

## **Build**

Run this command to build the package:

```bash
colcon build --merge-install --packages-select imp_msgs imp_ros_network_stats  --cmake-args -DBUILD_TESTING=OFF
```

## **Run**

### Basic Listener & Talker

This runs `talker` and `listener` ROS 2 nodes:

```bash
# Open new terminal
ros2 run imp_ros_network_stats talker --config <CONFIG_PATH> --namespace <ROS_NAMESPACE>
```

```bash
# Open new terminal
ros2 run imp_ros_network_stats listener --config <CONFIG_PATH> --namespace <ROS_NAMESPACE>
```

## **Verify**

### Basic Listener & Talker

When executed correctly, strings should be printed to the terminal similar to what is shown below:

```bash
# In terminal running talker
[INFO] [1686355951.476789684] [talker]: I sent msg: [1], Timestamp: 1686355951.476276398 seconds
[INFO] [1686355951.576812358] [talker]: I sent msg: [2], Timestamp: 1686355951.576375008 seconds
[INFO] [1686355951.676752000] [talker]: I sent msg: [3], Timestamp: 1686355951.676313877 seconds
```

```bash
# In terminal running listener
[INFO] [1686356196.887324511] [listener]: I heard msg: [1], Delay: 0.0005 seconds
[INFO] [1686356196.987503786] [listener]: I heard msg: [2], Delay: 0.0005 seconds
[INFO] [1686356197.087425096] [listener]: I heard msg: [3], Delay: 0.0006 seconds
```