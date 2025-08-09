# ROS2 Log Dashboard

A real-time web dashboard for monitoring ROS2 logs and system status across multiple nodes.

## Features

- Real-time log streaming via WebSockets
- Multi-node monitoring (Manual Control, Motor Controller, Object Detection, Camera)
- Interactive log filtering and search
- Visual status indicators and gauges
- Export functionality
- Responsive web interface

## Nodes Monitored

1. **Manual Control Node** - `/manual_control_status`, `/cmd_vel`
2. **Motor Controller Node** - `/motor/left_speed`, `/motor/right_speed`  
3. **Object Detection Node** - `/object_info`, `/camera/image_raw`, `/image_annotated`
4. **Servo Controller Node** - `/vector3_cmd`

## Installation

```bash
pip install -r requirements.txt

```
## Running

```bash
python3 ros2_log_server.py

```