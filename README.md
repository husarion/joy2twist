# Joy2Twist

Dockerized ROS node allowing control of ROS-powered mobile robots with Logitech F710 gamepad. Joy2Twist node is converting `sensor_msgs/Joy` message to `geometry_msgs/Twist` in order to provide velocity commands for the mobile robot. Therefore this package is compliant (but not supported by Husarion) with any other gamepad controller which is able to publish the `sensor_msgs/Joy` message. 

## Setup joy

Connect joy via nano USB receiver and make sure it is in **DirectInput Mode** (switch in front o the pad with letters **D** and **X**, select **D**).

To test if joy works, use `jstest /dev/input/js0`.
If the output is:

```
jstest: No such file or directory
```

See `ls /dev/input | grep js` and find your joy number. If it differs, apply changes in *compose.yaml* and launch file.

## Button mapping

| Button |     Function      |
| :----: | :---------------: |
|  `LB`  |  enable driving   |
|  `RB`  | slow driving mode |
|  `RT`  | fast driving mode |

If neither `RB` nor `RT` is pressed, the robot operates in *regular* driving mode.

To drive robot use sticks.
By default, linear `X` and `Y` are held by the left stick. Angular `Z` is controlled with the right stick.

### Emergency stop

| Button |      Function       |
| :----: | :-----------------: |
|  `A`   |    Reset E-stop     |
|  `B`   |   Trigger E-stop    |
|  `LT`  | Enable E-stop reset |

> **NOTE**: Handle of robot's emegency stop is available only when `~e_stop/present` parameter is set true. This funcionality will work with any robot configured as follow:
> - publishes robot's E-stop state uisng ROS topic of type `std_msgs/Bool`.
> - allows resetting robot's E-stop using ROS service of type `std_srvs/Trigger`.
> - allows triggering robot's E-stop using ROS service of type `std_srvs/Trigger`.
> Topic and services names can be configured using ROS parameters, see [Parameters](#parameters) for more info.

---
## ROS node API

ROS node is translating `/joy` topic to `/cmd_vel` topic.


### Publish

- `/cmd_vel` *(geometry_msgs/Twist)*

### Subscribe

- `/joy` *(sensor_msgs/Joy)*

### Parameters

Following parameters change joystick axes mapped to given robot axes of freedom. For more information about parameter values, refer to the joy package [wiki page](http://wiki.ros.org/joy#Logitech_Wireless_Gamepad_F710_.28DirectInput_Mode.29).

- `~axis_linear_x`      *(int, default: 1)* 
- `~axis_linear_y`      *(int, default: 0)*
- `~axis_angular_z`     *(int, default: 2)*

The robot can be operated at 3 scales of speed depending on pressed buttons. It's possible to adjust velocity scaling factors using a [config file](./joy2twist/config/joy2twist.yaml). The Units are m/s for linear movement and rad/s for angular movement.

- `fast`    *(float, default: 1)*
- `regular` *(float, default: 0.5)*
- `slow`    *(float, default: 0.2)*

The node can be configured using parameters described below to work with robots equipped with an E-stop interface. An example configuration for a robot with an E-stop interface can be found in [panther config file](./joy2twist/config/joy2twist_panther.yaml).

- `~e_stop/present`         *(bool, default: false)*
- `~e_stop/topic`           *(string, default: e_stop)*
- `~e_stop/reset_srv`       *(string, default: e_stop_reset)*
- `~e_stop/trigger_srv`     *(string, default: e_stop_trigger)*

## Docker image

[![Build/Publish Docker Image](https://github.com/husarion/joy2twist/actions/workflows/build-docker-image.yaml/badge.svg)](https://github.com/husarion/joy2twist/actions/workflows/build-docker-image.yaml)

| ROS distro | Supported architectures                      |
| ---------- | -------------------------------------------- |
| `melodic`  | `linux/amd64`, `linux/arm64`, `linux/arm/v7` |
| `noetic`   | `linux/amd64`, `linux/arm64`                 |

Available on [Docker Hub](https://hub.docker.com/r/husarion/joy2twist/tags)

### Demo

#### Controlling ROSbot 2 with a Logitech F710 gamepad

1. Clone this repo on your ROSbot:

    ```bash
    git clone https://github.com/husarion/joy2twist.git
    cd joy2twist/
    ```

2. Create `demo/.env` based on `demo/.env.template` file and modify it if needed (see comments)

    ```bash
    #SBC <> STM32 serial connection. Set:
    #SERIAL_PORT=/dev/ttyS1 # ROSbot 2
    #SERIAL_PORT=/dev/ttyS4 # ROSbot 2 PRO
    SERIAL_PORT=/dev/ttyAMA0 # ROSbot 2R
    ```

3. Launch on ROSbot

    Go to the `joy2twist/demo` folder and run:
    
    ```bash
    cd joy2twist/demo
    docker compose -f compose.rosbot.yaml up
    ```
