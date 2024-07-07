# Deimos Software Trials - Task 3_A
This repo contains the files of task 3 A that is writing code for aruco detection in a video and in gazebo world using OpenCV.

## Aruco detection in video
The repo contains a aruco.py python file and aruco.mp4 video that is fed to the aruco.py file to detect aruco markers in that video, the python script detects the markers by enclosing them in boxes along with marking the top left corner and writing the id.
### Explaination of the code 
- #### Import the libraries
  We will be using opencv-contrib-python library that has packages to detect aruco markers so install it by using the command line
  ```
  pip install opencv-contrib-python
  ```
  It will install the latest version of Opencv.<br>
  Once it is installed you can import it in your python script. The following imports are necessary for aruco detection
  ```
  import cv2
  import cv2.aruco as aruco
  ```
>[!NOTE]
>If you have opencv installed in place of opencv-contrib-python then first uninstall opencv and then install opencv-contrib-python

- #### Reading the Video
  ```
  cap=cv2.VideoCapture(r"<path to video>/aruco.mp4")
  while True:
    res,frame=cap.read()
    if not res:
        print("can't open file")
        break
  
    # code for detection
  
    cv2.imshow("frame",frame)
    cv2.waitKey(1)  
  ```
  The `cv2.VideoCapture()` gets the video from its path it can also access web-cam if index of camera is passed instead of path. `.read()` method reads the video frame by frame, it also gives a boolean True or False based on whether the video is read or not. `.imshow()` shows the frame in a window having  a title and `.waitkey()` sets the delay in reading the next frame for smooth transition.
- #### Detect Aruco Markers
  ```
  dic=aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
  params=aruco.DetectorParameters()

  while True:
    res,frame=cap.read()
    if not res:
        print("can't open file")
        break
    
    (corners,mark_id,rejected)=aruco.detectMarkers(frame,dic,parameters=params)

    # draw ids and boxes around detected markers
  
    cv2.imshow("frame",frame)
    cv2.waitKey(1) 
  ```
  OpenCV has a function `detectMarkers` that takes parameters
   1. Image to detect markers on i.e., video frame for a video
   2. Dictionary from which the aruco marker belongs, it depends on its size
   3. Parameters used in detection like removing perspective, checking contour etc. It's recommended to take default parameters.

  The `detectMakers` function also returns three values, Corners, Id and rejected (corners of the shapes rejected for being an aruco marker)

- #### Drawing around Markers
  If the function detects a marker it will return the corners matrix always in an anti-clockwise order from top left to bottom left. We can use these points to show boundaries, top left corner (to know if the aruco is rotated) and id on the aruco. `cv2.line()`, `cv2.rectangle()` and `cv2.putText()` can be used to draw lines, rectangles and show text respectively.

Watch working_aruco.mp4 in the repo to see the working video of the code
## Aruco Detection in Gazebo
For detecting Aruco markers in Gazebo first create a world with Aruco markers. This repo uses the models of markers given [here](https://github.com/sacchinbhg/gazebo_aruco_models.git). You can add the markers in `models` folder under `turtlebot3_gazebo` package.

### Creating World with Aruco Markers
You can launch any world in gazebo then go to Insert and place the marker model where you want. After adding the markers you can either save the world or include them manually in world file by adding
```
<include>
    <uri>model://marker_1</uri>
    <pose>-1.084460 4.612212 0.523755 0 0 -1.554291</pose> 
</include>
```
Get the pose estimates in Gazebo by World-->Model-->marker_model-->pose it will give the pose of the marker where you have placed it
### Writing Script to detect Marker
Inside turtlebot3_gazebo package you can make `scripts` directory and write a subscriber node that subscribes to `/camera/rgb/image_raw` topic. This topic publishes the images from camera of turtlebot3 as it moves ahead by `tele-op` command. The script will take the data, convert it to be accessible by opencv, then detect aruco markers and show the data or image.
### Explaining the Code
- #### Import the libraries
  CvBridge allows to convert ros images to opencv format and vice versa. Install it using the command
  ```
  apt-get install ros-noetic-cv-bridge
  ```
  Then import the following libraries in python script
  ```
  #!/usr/bin/env python3

  import rospy
  import cv2
  import cv2.aruco as aruco
  from sensor_msgs.msg import Image
  from cv_bridge import CvBridge
  ```
  The topmost line is written so that ros runs this file as a python file
- #### Initialising node
  ```
  rospy.init_node('aruco_detect')
  rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
  rospy.spin()
  ```
  This will initialise our subscriber node that will run `callback` function till it's aborted.

- #### Writing Callback Function
  We need to specify the dictionary and parameters for aruco detection outside the callback function because they are not gonna change their values
  ```
  dic=aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
  params=aruco.DetectorParameters_create()
  ```
  The callback function will take ros image from the topic so we need to change it using `CVBridge()`
  ```
  def callback(data):
      bridge = CvBridge()
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

      #opencv aruco detection code
  
      cv_image=cv2.resize(cv_image, (660, 440)) 
      cv2.imshow("cv_image",cv_image)
      cv2.waitKey(1)
  ``` 
  The `resize()` function will not let the cv_image window to take the whole screen and `imshow()` will show the image with detected aruco markers
- #### Launching the node
  You can run this node separately after launching the world by using
  ```
  rosrun turtlebot3_gazebo aruco_detect.py
  ```
  Or you can launch it along with the world by adding the following in the launch file of the world before closing the <launch> tag
  ```
  <node name= "aruco_detect" pkg= "turtlebot3_gazebo" type="aruco_detect.py" output="screen" required = "true"/>
  ```
  This command will file the node `aruco_detect` inside package `turtlebot3_gazebo` and run it

## How to Use this Repo
Clone the repo in src directory of your workspace then run `catkin_make` to build the catkin workspace and then source the workspace
```
$ cd ~/task3_ws/src/
$ git clone https://github.com/bhumii-ka/Deimos_task3_A.git
$ cd ~/task3_ws && catkin_make
$ echo "source ~/task3_ws/devel/setup.bash" >> ~/.bashrc
```
Then launch the turtlebot3_house world where aruco markers are installed
```
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
Then tele operate the bot using following command in new terminal to detect markers.
```
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
You can also see the marker id in the terminal you launched the gazebo world.


## Links
[Aruco detection in Gazebo World](https://youtu.be/at4K2Owzty8)
