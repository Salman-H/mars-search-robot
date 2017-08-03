# Mars Search Robot
Autonomous search and retrieval of samples of interest on a simulated Martian terrain using Python, OpenCV, and Unity.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/auto_mode_4.png" alt="" width="98%">

<a id="top"></a>
### Contents
1. [Introduction and Motivation](#1.0)
2. [Environment Setup](#2.0)
3. [Theoretical Background](#3.0)
4. [Design Requirements](#4.0)
5. [Design Implementation](#5.0)
7. [Testing and Review](#6.0)

------------

### Terminology
* **STMD** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Space Technology Mission Directorate of NASA
* **MSL** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Mars Science Lab: a robotic space probe mission to Mars launched by NASA in 2011
* **MRO** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Mars Reconnaissance Orbiter: a NASA satellite in Mars orbit
* **ROI** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Regions of Interests: Pixels identifying a distinct object or region in an image
* **RGB** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Red, Green, Blue: a color model consisting of the three additive primary colors
* **HSV** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Hue, Saturation, Value: cylindrical-coordinate representations of points in an RGB color model
* **FSM**  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Finite State Machine: an abstract machine that can be in one of a finite number of states at any time
* **OpenCV** &nbsp; Open-Source Computer Vision: a software library aimed at real-time computer vision

------------

<a name="1.0"></a>
<!--<div style="text-align:left;">
<span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;">1. Introduction and Motivation</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 1. Introduction and Motivation
This project originated from Udacity's [Search and Sample Return](https://github.com/udacity/RoboND-Rover-Project) challenge, which, in turn is based on the [**NASA** Sample Return Robot Challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) sponsored by the Space Technology Mission Directorate.

##### Objective
The objective of the NASA challenge and of this project is to demonstrate an autonomous capability to locate and retrieve specific samples of interest from various locations over a wide and varied terrain and return those samples to a designated zone in a specified amount of time with limited mapping data and high fidelity. The aim is to get first-hand experience with the three essential elements of robotics: perception, decision-making and actuation. The robot software is written in Python and the project is carried out in a simulator environment built with the Unity game engine.

##### Relevance
Before discussing any design metrics in detail, it is prudent to first at least, momentarily, appreciate the importance of retrieving samples from alien worlds. Towards this end, NASA’s Mars rover, *Curiosity*, will be used as an example:

Curiosity rover was deployed to Mars in 2012 as part of the Mars Science Lab and its goal is to access past and future habitability on mars. One way in which this is achieved is by utilizing the on-board X-ray Spectrometer to investigate the elemental composition of rock samples. For instance, to determine the inventory of the chemical building blocks of life such as carbon, hydrogen, oxygen and organic compounds. Within one year into its mission, Curiosity had already determined that ancient Mars could have been hospitable to microbial life [[link](http://science.sciencemag.org/content/343/6169/386)]. Bottom line: if any evidence of a second, independent origin of life is found, then that would be a profound discovery as it would immediately imply that life is common in the universe and not just specific to Earth! Furthermore, investigating the elemental composition of rock samples is also important to plan and design life-support systems for future manned missions. According to University of Boulder's [Center for Science and Technology Policy Research (CSTPR)](http://sciencepolicy.colorado.edu/), the exploration of Mars has so far shown that the key life-support compounds of O<sub>2</sub>, N<sub>2</sub>, and H<sub>2</sub>O are available and that the soil can be used for radiation shielding.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/mars_mosaic_1.png" alt="" width="98%">

In above images in the top row, MRO captures curiosity's successful 2012 entry into Martian atmosphere and tracks its landing and first movements. In the bottom row, curiosity takes a self-portrait and captures a photo of earth that puts things in perspective.

------------

<a name="2.0"></a>
<!--<div style="text-align:left;">
  <span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;"> 2. Environment Setup</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 2. Environment Setup
##### Robot Simulator
The simulator build can be downloaded from these links: [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](  https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip).

The *Training mode* can be used to get familiarized with the simulator and for recording sensor data:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/sim_screenshot.png" alt="" width="98%">

##### Package Dependencies
The project requires Python 3 along with following dependencies:
* [Flask](http://flask.pocoo.org/)
* [Numpy](http://www.numpy.org/)
* [Jupyter](http://jupyter.org/install.html)
* [Socketio](https://pypi.python.org/pypi/python-socketio)
* [Eventlet](http://eventlet.net/)
* [Matplotlib](https://matplotlib.org/users/installing.html)
* [OpenCV 2](http://opencv.org/)
* [Python Imaging Library (PIL)](http://www.pythonware.com/products/pil/)

Most of these dependencies can be resolved with Anaconda, an open-source python distribution and package manager aimed at Data Science. Its mini-version, Miniconda can also be used:
* [Anaconda](https://www.continuum.io/anaconda-overview)
* [Miniconda](https://conda.io/miniconda.html)

Alternatively, the following pre-built virtual environment can be used along with Anaconda or Miniconda:
* [RoboND-Python-Starterkit](https://github.com/ryan-keenan/RoboND-Python-Starterkit)

------------

<a name="3.0"></a>
<!--<div style="text-align:left;">
<span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;">3. Theoretical Background</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 3. Theoretical Background
The following theoretical concepts are used in this project: 
* Perception and computer vision principles including color thresholding and perspective transforms
* Geometric transformations of coordinate reference frames using rotation and translation matrices
* The use of finite state machines to design decision trees for autonomous actions

#### 3.1 Computer Vision Concepts
##### Color Thresholding
The primary sensor on the rover is a front-mounted camera that reads in color images. In order to autonomously drive the rover, it should be able to 'see' the area where it can drive (navigable area). For the purposes of this project, it just so happens that the sand on the ground throughout the simulated Martian environment is lighter in color compared to rest of the environment. A driving strategy can then be developed, such that, in order to determine where to drive, pixel locations with lighter colors can be found as a first step.

An incoming camera image can be stored as an array and operations performed on it to manipulate the image. The camera images have a dimension of 320x160 pixels. These can be stored as 8-bit unsigned integer [Numpy](http://www.numpy.org/) arrays (uint8) where the size of the array is (160, 320, 3) meaning the image size is 160 pixels in the y-direction (height), 320 pixels in the x-direction (width) and it has 3 layers or "color channels". These three color channels of the image are red, green and blue or "RGB" for short. The combination of intensity values (0 to 255) across the three channels determines what color is seen in the image. Here, a given camera image is displayed separately on each of its Red, Green and Blue color channels:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/color_channels.png" alt="" width="99%">
  
It can be observed from the above images, that, while the mountains are relatively dark (low intensity values) in all three color channels, both the ground and the sky are brighter (higher intensity) in the red, green and blue channels. However, in all cases it looks like the ground is a bit brighter than the sky, such that it should be possible to identify pixels associated with the ground using a simple color threshold. For instance, with a threshold of R,G,B = 160, only ground (navigable) terrain pixels can be extracted from the following 3-channel camera image into a single-channel or binary image:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/theory_color_thresh.png" alt="" width="99%">

##### Perspective Transforms
A perspective transformation is applied to convert an image viewed from the rover camera from a 3D view to a top-down 2D (warped) view. This is needed so that images viewed from rover camera can be eventually mapped to a ground truth of the world.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/grid_warping.png" alt="" width="99%">

The example grid image above is used to choose 4 source coordinates corresponding to the locations of the corners of the grid cell which is in front of the rover. Each grid cell represents a 1 square meter grid in the simulator environment. These four source points are subsequently mapped to four corresponding grid cell points in the output *warped* image where each 10x10 pixel square represents 1 square meter viewed from top-down.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/cam_to_rover_vision.png" alt="" width="95%">

Eventually, the warped, 2D top-down image is thresholded to extract a particular ROI (navigable terrain in above image), and then expressed in the rover's coordinate frame such that the ROI is centered at the origin of this coordinate frame. Fixing a coordinate system with respect to the robot is central to many robotic applications as it allows objects in the robot's environment to be described with respect to the robot, specifically, with respect to the robot's camera in this case.

#### 3.2 Transformation of Coordinate Frames
Ultimately, however, the ROIs in the rover's environment need to be on the same coordinate frame used to describe the position of the rover. This coordinate frame is referred to as the global or world frame. Expressing the ROIs in the world coordinate frame allows them to be mapped to a known world map.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/rf2wf_map.png" alt="" width="98%">

In the above image, the *Rover coordinate frame* is transformed to the *World coordinate frame* by first [**rotating**](https://en.wikipedia.org/wiki/Rotation_matrix) the Rover frame by **yaw angle**, and then [**translating**](https://en.wikipedia.org/wiki/Translation_(geometry)) it with a displacement equal to the **rover position vector**. The rotation is required to account for the fact that when the camera takes a picture, the rover can be pointing in any arbitrary direction, given by it's yaw angle. The translation is to account for the fact that the rover may be located at any position in the world when it takes a picture.

#### 3.3 State Machines
A state machine or a finite-state machine (FSM), is an abstract machine representing a mathematical model of computation. It can be in exactly one of a finite number of states at any given time. The state that it is in at any given time is called the *active* or the *current* state. Each state performs a set of **actions**. The state machine can change from one active state to another in response to external inputs called **events**. The change from one state to another is called a **transition** or a **switch**. Each state may also have an associated *handler* that supervises transitions or switching from that state.

A state machine is defined by
* The set of **transitions** governed by individual state handlers that determine the next states from the active state
* The set of **events** that can trigger a transition or switch from one state to another
* The set of **actions** that are performed in a given state

**NOTE:** An *event* answers the question, *"what just happened?"*, whereas a *state* answers the question, *"What to do now?"*. To be *in a state* is to be performing some particular action. We *listen* for events and *switch* to a state of action. Often, the terms *'state'* and *'action'* are used interchangeably. Therefore, rather than using the terms *'state'* or *'action'*, It can sometimes be helpful to instead use the term, *'a state of action'* in order to avoid any confusion.

------------

<a name="4.0"></a>
<!--<div style="text-align:left;">
  <span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;">4. Design Requirements</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 4. Design Requirements
##### Primary Metrics
The primary metrics of interest are as follows:

* Percentage of environment mapped
* Mapping Fidelity
* Number of rocks located
* Number of rocks collected
* Total time taken to complete mission

##### Minimum Criteria
The bare minimum criteria is to map at least *40%* of the environment at *60%* fidelity and locate at least one of the rock samples (not required to collect). Each time the simulator is launched in <span style="color: firebrick">Autonomous mode</span>, there will be 6 rock samples scattered randomly about the environment and the rover will start at a random orientation in the middle of the map.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/min_requirements.png" alt="" width="66%">

##### NASA Criteria
The objective in the NASA Sample Return Challenge is not only to locate samples of interest but also to pick them up and return them to the starting point! In this project, the same objective is pursued while aiming to map the entire environment with high fidelity in the least possible amount of time.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/rock-pickup2.gif" alt="" width="65%">

##### Ideal Outcome
In the best case scenario, the entire environment is mapped at a very high fidelity and all six rock samples are located and collected in the minimum total amount of time. To achieve this, the above metrics need to be optimized. Particularly, accuracy of the mapping analysis and the efficiency with which the environment is traversed.

------------

<a name="5.0"></a>
<!--<div style="text-align:left;">
  <span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;">5. Design Implementation</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 5. Design Implementation
Please note that all python code conforms to [PEP8 guidelines](https://www.python.org/dev/peps/pep-0008/).

The following short forms and abbreviations are used in this section:

* pixpts &nbsp;&nbsp; *pixel points*
* ROI &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *regions of interest*
* nav &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *navigable terrain pixels*
* obs &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *obstacle pixels*
* rock &nbsp;&nbsp;&nbsp;&nbsp; *rock pixels*
* pf &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *perspective frame*
* rf &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *rover frame*
* wf &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *world frame*

#### 5.1 Jupyter Notebook Analysis
Perception techniques are developed and tested on sensor data in the notebook file, *Rover_Project_Test_Notebook.ipynb*.
The three ROI pixels for the purposes of this project are those that represent navigable (ground) terrain, obstacles (rocks and mountains), and gold rock samples. These are respectively referred to in the code as *nav*, *obs*, and *rocks* for short.

#### 5.1.1 Identification of ROI Pixels from Rover Camera
In the *color_thresh* function, three different color thresholds are applied to incoming images from the rover camera to extract threshed, binary images identifying each ROI.

##### Gold rock samples
To recognize gold rock samples in images, the following HSV color range is used to capture the variation of gold color in rock samples from calibration data:
* lower bound = (75, 130, 130)
* upper bound = (255, 255, 255)

Since the input camera images are in the RGB color format, they are converted to the HSV format and then thresholded on this range to only extract pixels identifying gold rocks:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/hsv_thresh_new.png" alt="" width="98%">

##### Navigable Terrain and Obstacles
To recognize navigable (ground) and non-navigable (obstacles) terrain, an RGB threshold of (160, 160, 160) is used above which only ground pixels are detected and below which only non-ground (rocks/mountain) pixels are detected. To achieve this, two single-channel, numpy arrays of zeros are created, one for storing the threshed nav pixels and the other for storing the threshed obs pixels. These arrays have the same size as the input images. Then, to identify nav pixels, *those* indexes in the nav single-channel array are set to 1 that correspond to the indexes of the input 3-channel image where the RGB values are *above* the threshold. Similarly, to identify obs pixels, *those* indexes in the obs single-channel array are set to 1 that correspond to the indexes of the input 3-channel image where the RGB values are *below* the threshold.

The output of the <span style="color: purple">color_thresh</span> function with the above warped image as input is shown below. White pixels represent the ROI in question:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/color_thresh_io.png" alt="" width="99.9%">

#### 5.1.2 Mapping of ROI Pixels into a Rover Worldmap
A *SensorData* object reads data from the CSV sensor log file generated by the simulator. This file includes saved locations of camera snapshots and corresponding rover position and yaw values expressed in the world coordinate frame. The *worldmap* member variable is initialized with a size of 200 square grids corresponding to a 200 square meters space. This is the same size as that of the 200 square pixels *ground_truth* member variable. This allows for full range of output position values in x and y from the simulator.

The *process_image* function in the test notebook performs the following actions to process camera images:

1. Converts a 3D image into a warped 2D image with a perspective transform
2. Thresholds the warped image to identify the ROIs: nav, obs, rocks
3. Transforms the threshed image pixel points from perspective to rover frame
4. Transforms the threshed image pixel points from rover to world frame
5. Updates worldmap

##### Perspective transformations

1. The following source coordinates (expressed in the world frame) are defined corresponding to the locations of the four grid cell corners in the example grid calibration image:
  *  (x<sub>1</sub>, y<sub>1</sub>) = (14, 140)
  *  (x<sub>2</sub>, y<sub>2</sub>) = (301, 140)
  *  (x<sub>3</sub>, y<sub>3</sub>) = (200, 96)
  *  (x<sub>4</sub>, y<sub>4</sub>) = (118, 96)
2. Four destination points are defined corresponding to above four source points (see test notebook)
3. OpenCV function *getPerspectiveTransform* is used to get the *transform_matrix*
4. OpenCV function  *warpPerspective* is used to apply *transform_matrix* and warp a 3D camera image to a 2D top-down view

##### Color thresholding
The *color_thresh* function is used to apply three different thresholds to identify the three ROIs: nav, obs, rocks (see previous section).

##### Transformation of threshed ROI pixels from perspective to rover coordinate frame
This is done in the *perspect_to_rover* function. Pixel positions are expressed in the rover's coordinate frame such that the ROI is centered at the origin of this frame.

##### Transformation of threshed pixels from rover to world coordinate frame
This is done in the *rover_to_world* function. Pixel points are first geometrically rotated by rover yaw and then geometrically translated by rover x, y position on the world frame. Additionally, pixel points are clipped to be within world size of 200x200 pixels.

##### Updating worldmap with ROIs expressed in world coordinate frame
Recall that <span style="color: purple">worldmap</span> is defined as a 3-channel, empty, 2D numpy array of floats of size 200x200 in the <span style="color: purple">SensorData</span> class. Indexes of its Red, Blue, and Green color-channels corresponding to the indexes of world-frame pixel arrays representing obs, rock, and nav terrain respectively are updated with a max RGB value of 255:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/test_mapping.gif" alt="" width="98%">

#### 5.2 Autonomous Navigation and Mapping
#### 5.2.1 Implementing Perception
Perception is implemented in the module, perception.py. The same steps, as described in Section 5.1, are followed to convert camera images to ROIs expressed in the rover's frame:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/perception_step_1.png" alt="" width="99.9%">

In addition to the perception processes described in the previous section, the following modifications and improvements are made in this module:

* An effort has been made to replace magic numbers with named constants wherever possible
* All angles are expressed in degrees unless those needed for geometric transformation matrices
* Python *namedtuples* are sometimes used to express coordinates as a single variable to make their access more readable
* To be able to locate coordinates on the world e.g. those of the rover starting location, functions for inverse transformations are added to convert pixel points from the world frame to the rover's frame:
  * *inv_translate_pixpts*
  * *inv_rotate_pixpts*
  * *world_to_rover*
* The average angle from rover to navigable terrain pixels is divided into angles to the left and right of rover heading to derive a more robust wall-following strategy (see following section).
* To optimize mapping fidelity, following steps are taken to improve the accuracy of 3D to 2D conversion in perspective transforms:
  * In addition to angles, distances are also extracted from *to_polar_coords* and ROI pixels are trimmed such that only pixels within certain distances from rover are included
  * Worldmap is only updated when rover has a stable drive i.e. pitch and roll angles are within certain limits (these are optimized to increase fidelity without having to sacrifice percentage mapped too much)
  * Polar coordinates of rock samples are also stored as member variables of the *RoverTelemetry* class to assist in collecting rocks (see following section)

#### 5.2.2 Implementing Decision-making
Decision-making is implemented in the module, decision.py. This module implements a *state machine* with the help of the following additional modules:
* events.py &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; defines the set of events that trigger state transitions
* handlers.py &nbsp;&nbsp; determines the possible next states from the current active state
* states.py &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; defines the state actions that are performed when a state is active

For details on the state machine model, see Section 3.3.

##### State machine
The state machine used in decision-making is defined by the following set of *events*, *transitions*, and *actions*.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/state_machine.jpg" alt="" width="99%">

##### Wall following
As an alternative to implementing path-planning algorithms, a wall-following approach is used such that the rover traverses the terrain by keeping the mountains and large rocks to its left to ensure that it always visits an area once and doesn't lose its way. The strategy used for achieving this relies on the average angle from the rover to the nav terrain pixels (nav_angle). The basic idea is to add an offset to this angle to bias the rover heading such that it always follows the left wall.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/nav_angles_exp.png" alt="" width="99%">

Recall that he rover heading (where the rover points) is defined by the x-axis of the Rover coordinate frame. The wall-following strategy is improved and made more precise by qualifying the average of *nav_angles* a bit further; instead of just averaging the angles from the rover to all nav terrain pixels, only angles to those pixels are used that lie to the left of rover heading. This is because a biased rover heading obtained from offsetting the average of *nav_angles* will work in some scenarios but not in those where the nav terrain has significant deviation from the rover's current heading (as shown in the figure). However, since the average of *nav_angles_left* comparatively much better approximates the wall angle from the rover, a biased rover heading obtained from using this as an offset instead accounts for most wall-nav deviation scenarios, resulting in a more robust wall-following behavior. The *FollowWall* class governs the actions taken in this state while the *following_wall* function handles transitions from this state.

##### Collecting rock samples
The strategy for locating and collecting rock samples is devised in a way that it doesn't conflict with wall-following, where, the rover would lose the wall while going after a rock. To avoid such a situation, samples are only detected if they are nearby, so that the rover can easily find the wall again after collecting them. This is achieved by using a rover-to-rock distance limit beyond which rock samples are ignored. The *GoToSample* class governs the actions taken in this state while the *going_to_sample* function handles transitions from this state.

##### Getting unstuck
To detect if the rover has gotten stuck somewhere, a timer is started every time the velocity drops to 0. If the time exceeds the designated *stuck time* for a given state, the *GetUnstuck* state is activated. If sufficient velocity is reached during the unstuck maneuvers, the *getting_unstuck* handler switches to another state. The timer has to be switched off every time the rover transitions to a different state to account for the possibility of getting stuck in that state.

##### Returning home
The home world coordinates (99.7, 85.6) are expressed in the rover's frame using the *world_to_rover* function from the *perception* module, as described in the previous section. Home cartesian coordinates are then converted to polar coordinates and the distance and heading to home is tracked. The distance to home is used to vary the rover's speed and heading profiles as follows:

* When the rover is sufficiently far away from home, a pure nav heading is used
* When the rover gets closer to home, a weighted average of home and nav headings is used with a 3:7 ratio
* When the rover is fairly close to home, a pure home heading is used with a more controlled speed

The *ReturnHome* class governs the actions taken in this state while the *returning_home* function handles transitions from this state.

------------

<a name="6.0"></a>
<!--<div style="text-align:left;">
<span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;"> 6. Testing  and Review</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 6. Testing  and Review
The simulator's <span style="color: firebrick">Autonomous mode</span> is launched from the command line with the following call:

```python
python drive_rover.py
```

##### Mission criteria
The mission is deemed complete when the rover has collected all six rock samples and mapped at least 95% of the environment or time spent doing so has exceeded 680 seconds, whichever comes first.

##### Note on unstuck
It takes anywhere from 2 to 5 seconds for the *unstuck* routine to kick in if the rover gets stuck in a repetitive behavior, depending on the circumstances leading to that behavior.

#### 6.1 Mission Results
After multiple runs in the simulator, the following results are achieved:

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/demo_results.png" alt="" width="99.9%" align="">

* Environment mapped: &nbsp; *92 - 96 %*
* Mapping fidelity: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *> 80%*
* Rocks located: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *≥ 6*
* Rocks collected: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *≥ 6*
* Time taken: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *690 - 780 s*

**Note:** The simulator is run on a machine with the following specifications

* Operating System: &nbsp;&nbsp;&nbsp;&nbsp; *ubuntu 16.04 LTS 64-bit*
* Processor Type: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *Intel®  Core™ i7-7700*
* Processor Speed: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *3.60 GHz  x 8*
* Processor Cores: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *4*
* Processor Cache: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *8 MB*
* RAM: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *12 GB*
* Graphics: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *NVIDIA GeForce GTX 1050*
* Video Memory: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *2 GB*

**Note:** The autonomous mode is so far tested with the following simulator settings

* Graphics Quality: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *Fantastic*
* Screen Resolution: &nbsp;&nbsp;&nbsp; *1440 x 960*

#### 6.2 Improvements
In addition to the design improvements described in Section 5, the following enhancements can be made in the following areas:
##### Perception
Analysis of camera images can be enhanced to devise techniques for detecting more tricky obstacle situations, such as, rocks that protrude from the sides of the mountains and otherwise not detected in the current design.

##### Decision-making
There are a few redundancies in the actions defined in the states module which can be removed to make the state machine more versatile and maintainable. The current implementation of the state machine can also be improved by using the python *design pattern* for a state machine involving stacks. Ultimately, the wall-following approach should be replaced with a path-planning strategy e.g. involving Dijkstra's or A-star shortest-path algorithms.

------------
> Copyright © 2017, Salman Hashmi. See attached licence

