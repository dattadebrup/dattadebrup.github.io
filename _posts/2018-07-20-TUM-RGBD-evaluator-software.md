---
layout: post
title:  "TUM RGBD Evaluator Software"
author: debrup
categories: [ RGBD , evaluator ]
image: assets/images/evaluator.png
featured: true
---

I made this evaluator software while I was starting off to learn and write an visual odometry algorithm from scratch. So I was given the task by my mentor at [JdeRobot](https://jderobot.org/) to write a software at first which would ease the process of testing and visualising my odometry algorithm.

So I came up with this software which would help the user to concentrate only on his/her odometry/SLAM algorithm without worrying about how to access the data , test the performance , accuracy and visualising their algorithm. The software does all of these for the user.

This software works with the [TUM RGBD dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset "RGB-D SLAM Dataset and Benchmark") . The performance(speed) of the users' algorithm is tested on the basis of the **Real-time factor** and the accuracy of the algorithm is computed on the basis of **Absolute Trajectory Error**. Both these parameters are shown dynamically on the GUI . Also the predicted and actual(groundtruth) positions are visualized simultaneously on a graph widget for comparing them visually.

The software is open-sourced and available in my [Github Repo](https://github.com/dattadebrup/TUM-RGBD-odometry-evaluator "TUM-RGBD-odometry-evaluator") . Details instruction on how to download and run this software is provided there.


Here is a video showing the working of the software while running a **Monocular Visual odometry algorithm** (about which I have discussed in my [next](https://dattadebrup.github.io/monocular/inertial/odometry/2018/07/23/Monocular-Visual-and-Inertial-Odometry.html) blog post):

<iframe width="711" height="400" src="https://www.youtube.com/embed/2coEdSWuACA" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>



### Discussion of the API's of the software:-

The software has simple yet flexible API's to access datas from various sensors according to users' will from the **ROSbag** file of the dataset .

User's will be able to access various sensor's data according to their need.

**Sensors available:** - 'color_img' , 'depth_img', 'orientation' , 'accelerometer' , 'scan' (laser scan) .

 This is how the Rosbag file looks when displayed in the ```rqt_bag``` (ROS package) .

![](https://dattadebrup.github.io/assets/images/rosbag_image.png)

Let's assume that the user only wants to work with the Color(RGB) image and Depth image. Then the API for getting the sensor datas will be:

```data = self.getReadings( 'color_img' , 'depth_img' )```

So the sensor datas will be stored in the the ```data``` object.

The Color image can then be accessed as follows:-

```color_image = data.color_img```

and the depth image can be accessed as follows:-

```depth_image = data.depth_img ```

The timestamp of the Color Image can be accessed as follows:-

```color_image_t = data.color_img_t```

and the timestamp of the Depth Image as follows:-

```depth_image_t = data.depth_img_t```


**Similiarly ,** to get the readings of other sensors the required sensors name has to be mentioned while calling ```data = self.getReadings()```  seperated by commas (,). 

```data.color_img``` - for RGB color image and ```data.color_img_t``` for its timestamp.
```data.depth_img``` - for depth image and ```data.depth_img_t``` for its timestamp.
```data.accelerometer``` - for accelerometer data and ```data.accelerometer_t``` for its timestamp.

```data.orientation``` - for orientation data and ```data.orientation_t``` for its timestamp.

```data.scan``` - for laser scan data and ```data.scan_t``` for its timestamp.
(You can mention as many as sensors names during calling the method).

*  **So how does datas from multiple sensors are read and provided to the users by the ```self.getReadings()``` method?**

Let's assume that the user only wants the data from 'color_img' , 'depth_img' and 'scan' sensors. So he will call the method like this ```data = self.getReadings('color_img' , 'depth_img','scan')``` . Internally the program then starts to read the messages ROSbag file chronologically and if any of the above mentioned sensor topic is found , the topic data gets stored in its respective attribute.The program continues to read the topics in ROSbag file sequentially until all the sensor datas required by the user are read and stored in its respective attributes. And since the data frequency of different sensors are different so while reading the sensor datas sequentially the latest data from a particular sensor will override it's previous value.

**REMEMBER :** By this method only the sensor name mentioned while calling this method will have the sensor data . e.g. - In the above example (```data = self.getReadings('color_img' , 'depth_img','scan')``` ) only ```data.color_img``` , ```data.depth_img``` and ```data.scan``` will contain its' respective sensor data and other attributes like ```data.accelerometer``` and ```data.orientation``` will contain ```NoneType``` data.

There is also another way the user can access the sensor datas sequentially. 
By calling the method like this ```data = self.getReadings('stream')``` with the 'stream' keyword the method only reads and  provides a single sensor data as available in the Rosbag file chronologically. The name of the sensor who's value is available in that particular call can be obtained in the ```data.sensor``` attribute and the sensor data from can be obtained the sensor's respective attribute.
For example in a particular call ```data = self.getReadings('stream')``` the 'scan' data is read then ```data.sensor``` will be == ```'scan'``` and ```data.scan``` will contain the 'scan' sensor's data and ```data.scan_t``` will conatin its' timestamp.

## Data Types:
* The Color RGB image is provided in 640×480 8-bit RGB format.
* The Depth image is provided in 32-bit floating point precision.
* The laser scan data is provided in numpy array format.
* Orientation data can be accessed as follows:

	```qz = data.orientation['qz']```

    ```qw = data.orientation['qw']```
    
    'qx' and 'qy' are essentially zero(since it is a 2D odometry).
* Accelerometer data can be accessed as follows: 

	```ax = data.accelerometer['x']```

	```ay = data.accelerometer['y']```

	'ay' is essentially zero.
* The Timestamps are floating point numbers.
* For more details refer to the site of the TUM RGBD dataset <https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#ros_bag>.