
# ROS2 Simple Path Publisher & Subscriber

Basic route publisher and subscriber for ROS2 Galactic.

The polygon vehicle can follow the root with this package. 



## Dependencies
```bash
nav2-msgs
```
```bash
std-msgs
```
## Installation

Build and run package with following commands

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```
```bash
ros2 launch path_tracker path_launch.py 
```

## Parameters

#### Parameters in yaml file


| Parameter | Type     | Description                |
| :-------- | :------- | :------------------------- |
| `frequency` | `double` | Vehicle speed parameters in seconds  |
| `car_length` | `double` | Length of the car |
| `car_width` | `double` | Width of the car |
| `path_topic` | `string` | Subscribed path topic |
| `publish_vehicle_topic` | `string` | Publish vehicle topic |

    
## Screenshots

![Route Screenshot](https://media.discordapp.net/attachments/809713676486705172/1009804379211186257/Screenshot_from_2022-08-18_15-41-34.png?width=1173&height=660)


## Authors

- [@hasanzfdn](https://github.com/hasanzfdn)

