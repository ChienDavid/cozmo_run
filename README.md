# <p style="text-align: center;">COZMO_RUN</p>

By incorporating the visual feedback from the front camera with the PID control of the velocity of the robot, this package enables the cozmo robot to navigate on a certain track and avoid the obstacles on the track.

---

## Requirements  
Anki cozmo
Android phone/iPhone
Ubuntu 14.04/16.04  
ROS Indigo/Kinetic  
Python3.5  
(Cozmo SDK)  
Cozmo driver for ROS https://github.com/OTL/cozmo_driver  (Takashi Ogura)  
For more detail about Cozmo ros and the setup, please visit the link above
---

## ROS structure

### Nodes:

#### /obj_id (ctrl2.py)  
The node handles all the computer vision processing. It subscribes to the video feed and publishes the size and the location of each objects. It also reads a text file with calibration information.

`Subscribe`
* /cozmo_camera/image - The video feed from cozmo's head cozmo_camera
* /odom - The odometer reading of the cozmo



`Publsh`
* /head_angle - The topic adjusting the head angle for best vertical FOV setting (cozmo_driver)
* /obj_size - The topic containing road info, obstacle info and sign info



#### /vel_ctl (vel6.py)  
This node does the PID (PD in this case) control of the speed of the robot based on where the center of the road is and whether there are obstacles ahead.  

`Subscribe`
* /obj_size - The topic containing road info, obstacle info and sign info

`Publish`
* /cmd_vel - The topic for robot moving speed (cozmo_drover)


#### /img_cal  (img_cal.py)
The color in the video stream returned by the cozmo is very warm and the auto-white balance doesn't perform well either. So, the function of this node is to manually calibrate the white balance by capture the center half of the image with white object and write the calibration information to a text file.  

`Subscribe`
* /cozmo_camera/image - The video feed from cozmo's head cozmo_camera


## Other settings

1. The color of the road is set to be orange in this package. You can adjust the color by setting color_min and color_max in the ctrl2.py. You can change the setting to the stop sign in the same file. (obj_color_min/obj_color_max)
2. Be sure to change the stream setting in the cozmo driver so it outputs color stream but not black and white. (coz.camera.color_image_enabled = True)  
3. The frame rate setting can also be done by properly setting the cozmo driver  
4.
