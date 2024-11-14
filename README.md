# modbus_cfw500

`modbus_cfw500` is a ROS 2 package designed for interfacing with the WEG CFW500 inverter via Modbus RTU. This package reads important registers from the inverter and publishes their values as ROS topics for further use in robotic applications. It utilizes the `pymodbus` library for Modbus communication and `rclpy` for ROS 2 integration.

## Features

- Reads the following Modbus registers from the WEG CFW500 inverter:
  - Frequency (register 1000)
  - Output Speed (register 2)
  - Motor Current (register 3)
  - Output Voltage (register 7)
  
- Publishes the read data as ROS 2 topics:
  - `/cfw500_frequency` (Float32)
  - `/cfw500_output_speed` (Float32)
  - `/cfw500_motor_current` (Float32)
  - `/cfw500_output_voltage` (Float32)

- Built using ROS 2 and `pymodbus`.

## Installation

### Clone this repository into your ROS 2 workspace:


     cd ~/dev_ws/src
     git clone https://github.com/MarcuSpade/cfw500_modbus.git
### Install dependencies:

The package depends on `rclpy`, `std_msgs`, `pymodbus` and `pyserial`. You can install the necessary dependencies using:

    pip install pymodbus
    pip install pyserial
    
And then install ROS 2 dependencies by building the workspace:

  
     cd ~/dev_ws/src
     rosdep install --from-paths src --ignore-src -r -y


Source your workspace:
    
    source ~/dev_ws/install/setup.bash

## Usage

After building and sourcing your workspace, you can launch the node using the provided launch file:

     ros2 run cfw500_modbus cfw500_modbus_node 


This will start the cfw500_modbus_node and begin publishing data to the following topics:

    /cfw500_frequency
    /cfw500_output_speed
    /cfw500_motor_current
    /cfw500_output_voltage

## Viewing Topics

You can view the topics being published using the following command:

    ros2 topic list

To see the values published to a specific topic, use:
    
    ros2 topic echo /cfw500_frequency
    ros2 topic echo /cfw500_output_speed
    ros2 topic echo /cfw500_motor_current
    ros2 topic echo /cfw500_output_voltage

## Configuration

You can modify the serial port, baudrate, and other parameters for the Modbus connection by editing the cfw500_modbus_node.py file.

Example:

    self.client = ModbusClient(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

License

This package is licensed under the Apache License 2.0.
