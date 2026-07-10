# C-SLAM+: Fault Tolerant, Resource Optimized Collaborative Multi-Robot SLAM

## Overview

C-SLAM+ is a centralized multi robot SLAM architecture where individual robots
(ESP32 + LiDAR) send sensor data to a Raspberry Pi coordinator for map merging and
exploration planning. That design works well but has a single point of failure:
if the WiFi link or the coordinator itself goes down, robots lose the ability to
map, coordinate, or avoid duplicating exploration effort. 

C-SLAM+ adds:
- **Hybrid communication** — WiFi to the coordinator + ESP-NOW for direct
  robot to robot messaging, so robots aren't fully dependent on the central link
- **Local data buffering** — SD card logging during WiFi loss, replayed on
  reconnect, so no sensor data is lost during outages
- **Decentralized frontier allocation** — auction style bidding between robots
  over ESP NOW, so exploration decisions survive coordinator loss
- **Quadtree map representation** — reduces memory and data size for both local
  and merged global maps
- **Graceful degradation** — the system keeps exploring and avoiding collisions
  even when the coordinator is unreachable, without redundant hardware
## Motivation
 
 Most existing collaborative SLAM systems are validated on relatively capable,
 costly hardware. Decentralization and fault tolerance techniques from swarm
 robotics research have rarely been tested at the ultra low cost, resource 
 constrained tier this project targets. C-SLAM+ investigates how far those
 techniques can be pushed on cheap hardware, and what has to be adapted or
 dropped to make them fit.
## Hardware
 
| Component | Purpose |
|---|---|
| 2-wheel differential drive chassis | Robot base |
| Motor driver | Motor control |
| Wheel encoders (x2) | Odometry |
| ESP32 | Onboard controller (WiFi + ESP-NOW) |
| LD19 LiDAR  | Mapping sensor |
| BMI160 IMU | Odometry drift correction |
| MicroSD card module | Local data buffering |
| Raspberry Pi 5  | Central coordinator |
| Li-ion rechargeable battery + regulator | Power |
 
 
## Software Stack
 
- **Firmware:** ESP32 (Arduino), micro-ROS
- **Coordinator:** ROS2 , Ubuntu
- **SLAM:** slam_toolbox 
- **Map merging:** RANSAC, ICP, custom occupancy grid Bayesian fusion
- **Communication:** WiFi/UDP (DDS-XRCE), ESP-NOW

## Evaluation
 
Experiments compare this system against the original centralized baseline under
identical fault injection scenarios (WiFi loss, coordinator loss), measuring:
- Data loss during outages
- Recovery time on reconnect
- Map size before/after quadtree compression
- Exploration coverage over time

## Acknowledgments
 
Built as an extension of the original C-SLAM project developed at the
Department of Computer Science and Engineering, University of Moratuwa.

