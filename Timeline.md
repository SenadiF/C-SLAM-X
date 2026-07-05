# C-SLAM-X System Implementation Plan

## Hardware & Base Setup
- Build chassis  
- Wire motor driver, encoders, IMU, SD card module to ESP32  
- Calibrate encoders and IMU individually  
- Set up Raspberry Pi (OS, ROS2, micro-ROS agent installation)  

---

## Sensing and Communication
- Package scan + odometry into micro-ROS messages, confirm they arrive on Pi  
- Implement ESP-NOW between two ESP32 boards  
- Confirm ESP32 running WiFi (to Pi) and ESP-NOW (peer) simultaneously without conflict  

---

## Local SLAM
- Install SLAM package on Pi, one instance per robot  
- Verify each robot builds an accurate local map independently  

---

## Map Merging
- Implement RANSAC coarse alignment between two local maps  
- Implement ICP fine alignment  
- Implement Bayesian occupancy grid fusion for overlapping cells  
- Test merge quality with dynamic obstacles present  

---

## Quadtree Optimization
- Implement quadtree representation for occupancy grids  
- Benchmark memory/size savings vs raw grid  
- Confirm merge/SLAM logic still works correctly on quadtree maps  

---

## Frontier Exploration & Decentralized Allocation
- Implement frontier detection on occupancy grid  
- Implement bid calculation (distance based) on ESP32  
- Implement broadcast + "lowest bid wins" logic over ESP-NOW  
- Test two robots selecting different frontiers without central assignment  

---

## Fault Tolerance
- Implement SD card logging when WiFi/Pi link unreachable  
- Implement replay on reconnect to Pi  

---

## Full Pipeline End-to-End Check
- Integrate all modules  
- Test full multi robot SLAM + exploration pipeline  
- Validate stability under packet loss and dynamic environments  
