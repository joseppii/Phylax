# Phylax MKI

[//]: # (Image References)

[image1]: ./images/phylax.png "Rendered image" 

ROS Packages for Phylax, a 6wd autonomous vehicle based on the wild thumper platform.

![alt text][image1] 

## Description

The robot will be build around NVIDIA's AGX processing module, using a quad camera setup for surround view, classification and target tracking. SLAM will be implemented using a Velodyne VLP-16 LiDAR and standard ROS packages. NVIDIA's Deepstream framework will be used for all tasks related to image processing running on a custom pipeline application server. A Texas instruments AWR1642 mmwave radar will be used for enhanced target tracking and obstacle avoidance. 

## Advanced Features
* Continuously variable FOV video streaming
* Multicamera AI based threat classification and tracking

## Technical Characteristics

* Processing unit: NVIDIA AGX
* Lidar: Velodine VLP-16
* Radar: TI AWR1642
* IMU: Inversense ICM-20948
* GPS (rover): U-blox zed-F9P 
* GPS (base): EMLID REACH RS+
* Motor Controllers: Pololu SMC G2 18v12
* Motors: 34:1 Metal Gearmotor 25Dx67L mm HP 6V with 48 CPR Encoder x6 
* Primary remote controller (transmitter): FrSky X-lite 
* Primary remote controller (receiver): FrSky RX8R
* Secondary remote controller: Sony PS4 remote controller
* WiFi/BT:  Intel Wireless-AC 9260 
* Camera board: LI-JXAV-MIPI-ADPT-6CAM-FP
* Cameras: LI-IMX185-MIPI-CS/M12 x 4

## Performance

* Max speed: 7km/sec
* Uptime (avg): 50 minutes                                            
