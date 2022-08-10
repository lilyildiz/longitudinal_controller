
# ROS2 Longitudinal Controller

A basic PID Longitudinal Controller for ROS2 Foxy and SVL Simulator.


## Dependencies

`lgsvl-msgs`

## Deployment

To deploy this project run 
```bash
  ros2 launch longitudinal_controller Controller.launch.py
```
## Parameters


| Parameter | Type     | 
| :-------- | :------- | 
| `target_velocity` | `float` |
| `kp` | `float` | 
| `kd` | `float` | 
| `ki` | `float` | 
| `frequency` | `float` | 
| `odometry_topic` | `string` |
| `control_data_topic` | `string` |

## Screenshots
 
Test run of the package with 20 mps target velocity.

![Screenshot](https://cdn.discordapp.com/attachments/1006192399862730763/1006900999522357309/unknown.png)


## Authors

- [@lilyildiz](https://www.github.com/lilyildiz)

