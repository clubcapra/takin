# capra_lidar

Capra-Takin's **capra_lidar** package is used as interfacing with the Lidar.

## Dependencies

### Internal Dependencies

- sick_tim

## Overview

The capra_lidar package is only a wrapper of sick_tim.

For more details, go see the [sick_tim](http://wiki.ros.org/sick_tim) 

### Usage

```sh
$ roslaunch capra_lidar sick_tim.launch
```

### Published Topics

capra/scan

### Configuration

In our configuration, the Lidar is connected by a Ethernet Port. The Lidar needed to be on the same network as the listener.

The Lidar is configured on the 192.168.1.155. In case, you need to change the ipaddress, you need to change: 

- The lidar internal ipaddress with [Sick Software](https://www.sick.com/us/en/downloads/software?q=%3Atyp1%3AConfiguration%2520software%3Atyp2%3ASOPAS%2520ET%3ADef_Type%3ADownload) 
- The hostname inside the sick_tim launch file

