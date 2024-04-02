# Coresense Instrumentation

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![main](https://github.com/Juancams/coresense_instrumentation/actions/workflows/main.yaml/badge.svg?branch=main)](https://github.com/Juancams/coresense_instrumentation/actions/workflows/main.yaml)
[![codecov](https://codecov.io/gh/Juancams/coresense_instrumentation/graph/badge.svg?token=EvUIoImzzh)](https://codecov.io/gh/Juancams/coresense_instrumentation)

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/CoreSenseEU/coresense_instrumentation
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to coresense_instrumention
```

## Usage
### Coresense Instrumentation Driver
<details>
<summary>Click to expand</summary>

```python
def generate_launch_description():

    names = ['scan_raw',
             'nav_vel',
             'image_raw']

    topics = ['/scan_raw',
              '/nav_vel',
              '/head_front_camera/rgb/image_raw']

    msgs = ['sensor_msgs::msg::LaserScan',
            'geometry_msgs::msg::Twist',
            'sensor_msgs::msg::Image']

    node_types = ['Producer',
                  'Consumer',
                  'Producer']

    ns = ''

    composable_nodes = []
    for topic, msg, name, node_type in zip(topics, msgs, names, node_types):
        composable_node = ComposableNode(
            package='coresense_instrumentation_driver',
            plugin='coresense_instrumentation_driver::Instrumentation'
                    + node_type + '<' + msg + '>',
            name=name + '_node',
            namespace=ns,
            parameters=[{'topic': topic,
                         'topic_type': msg,
                         'type': node_type}],
        )
        composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='coresense_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )
```
You have to indicate the name you will give to your nodes, the topic you subscribe to, the type of message and whether you are going to subscribe or publish to the topic.

</details>

```shell
ros2 launch coresense_instrumentation_driver coresense_instrumentation_driver.launch.py
```

### Coresense Instrumentation Rviz
```shell
ros2 launch coresense_instrumentation_rviz coresense_instrumentation_rviz.launch.py
```

<details>
<summary>Click to expand</summary>

  
![image](https://github.com/Juancams/coresense_instrumentation/assets/44479765/fc2403ed-f39d-4b4d-ac86-57477d56342e)
![image](https://github.com/Juancams/coresense_instrumentation/assets/44479765/186e220d-ec4b-4071-9ecd-bc408d6e9725)

When you launch rviz, you will see on your right a panel like this, but with your nodes. Through the following buttons, you can activate/deactivate your nodes and create/delete your publishers/subscribers
</details>

## Demo
[instrumentation_driver_demo](https://github.com/Juancams/coresense_instrumentation/assets/44479765/e6cada5c-5071-4a41-b226-5dc5c18a37aa)
