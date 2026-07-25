# Week 3


## Completed Tasks
- Calibrated the IMU using the FastIMU library  
- Started working on completing a single functioning robot 
- Changed the message type of the encoder readings to an already existing package since custom messages aren't supported in arduino 
- Change in the architeccture - Without sending the raw data to the pi from the esp32 get the readings and decode it at the esp itself and publish the scan topic using micro ros to the pi 


## Learnings 
- If the wi-fi is down or the connection Between the esp32 and pi are down explored the idea of a state machine as follows 
Normal mode - When connected with pi 
Local_exploration mode - Try to reach the last given goal if it cannot be reached explore using the follow the gap algorithm (To be studied more )
Sync mode - When it reconencts and gives the explored data 

## Still to be completed 

- Do the full setup for one robot and generate the map and view it from rviz 
