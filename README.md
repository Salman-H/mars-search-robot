# Converting a Land Rover Into a Search Robot Using a Camera
<!--# Converting a Land Rover Into a Search Robot With Only a Camera
 -->

#### Autonomous search and retrieval of samples of interest on a simulated Martian terrain using Python, OpenCV, and Unity.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/title-displays-1.webp" alt="" width="94%">
<br>
<sup>Credit: Udacity</sup>
</p>
<!--auto_mode_4.webp-->

<p align="left">
👨🏻‍💻 Salman Hashmi   ✉️ sal.hashmi@pm.me   🗓️ Updated March 3, 2024
</p>

> <sub>Assumes basic mathematical and programming knowledge.<br>If you're seeing this in dark mode, consider disabling it to optimally view illustrations.</sub>

---------

### Retrieving Samples From Alien Worlds

NASA's [Curiosity Rover](https://en.wikipedia.org/wiki/Curiosity_(rover)) landed on Mars in 2012 to assess the red planet's past and future habitability. The rover used its onboard [X-ray spectrometer](https://en.wikipedia.org/wiki/X-ray_spectroscopy) to investigate the elemental composition of Martian rock samples to see if they contained any chemical building blocks of life such as carbon, hydrogen, oxygen, and organic compounds. Only a year into its mission, Curiosity had already determined that [ancient Mars might have supported microbial life]((http://science.sciencemag.org/content/343/6169/386)). The [Perseverance Rover](https://en.wikipedia.org/wiki/Perseverance_(rover)), which followed Curiosity in 2020, has since also found rock samples from an [ancient river delta where life may have once existed](https://www.scientificamerican.com/article/perseverance-mars-rover-makes-fantastic-find-in-search-for-past-life/).

And while this is not direct evidence to support life beyond Earth, these are immensely important discoveries. More concrete evidence could eventually confirm that life is not specific to Earth but is common in the universe!

<p align="left"; style="line-height: 60% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/mars-landing-bd.webp" alt="" width="98%">
<br>
<sup>  The Mars Reconnaissance Orbiter, a NASA satellite in Mars orbit, captures Curiosity's successful 2012 entry into the Martian atmosphere and tracks its landing and first movements. (Credit: NASA/JPL-Caltech)</sup>
</p>

<p align="left">
<figure>
  <img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/mars-landing-bd.webp" alt="" width="98%">
  <figcaption>The Mars Reconnaissance Orbiter, a NASA satellite in Mars orbit, captures Curiosity's successful 2012 entry into the Martian atmosphere and tracks its landing and first movements. (Credit: NASA/JPL-Caltech)</figcaption>
</figure>
</p>

Besides assessing past habitability, exploring the elemental composition of samples is also important to plan and design life-support systems for future manned missions. According to the University of Boulder's [Center for Science and Technology Policy Research (CSTPR)](http://sciencepolicy.colorado.edu/), this exploration has so far shown that the key life-support compounds of oxygen, nitrogen, and hydrogen are present on Mars, and its soil can provide radiation shielding.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/mars-selfie-bd.webp" alt="" width="98%">
<br>
<sup>  Curiosity takes a self-portrait (selfie) and captures a photo of earth that puts things into perspective. (Credit: NASA/JPL-Caltech)</sup>
</p>

### The Gist of Robotics

To find innovative approaches to the problem of autonomous search and retrieval, NASA's Space Technology Mission Directorate (STMD) initiated the [Sample Return Robot Challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html). Its stated objective is to demonstrate an autonomous capability to locate and retrieve specific samples of interest from various locations over a wide and varied terrain and return those samples to a designated zone in a specified amount of time with limited mapping data and high fidelity.

In this project, we will undertake a simplified version of this challenge based on Udacity's [Search and Sample Return](https://github.com/udacity/RoboND-Rover-Project), which uses the NASA challenge as a pretext to demonstrate the gist of robotics: perception, decision-making, and actuation. We will get a high-level understanding of—and first-hand experience with—these concepts to see how they can enable autonomy, with a particular emphasis on perception, also known as [computer vision](https://en.wikipedia.org/wiki/Computer_vision).

### Setting Up the Simulation Environment

> [!NOTE]
> A *rover* is a land vehicle. A *robot* is an autonomous mechanical machine. A rover may or may not be a robot. It *can* be a robot if programmed for autonomy—like in this project. So, the terms rover and robot may be used interchangeably.

The project is carried out in a simulation environment built with the [Unity game engine](https://unity.com/). You can download the simulator from the following links, depending on your OS: 

<p>
<kbd>
  <a href="https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip" title="Download for Windows"><img src="https://img.shields.io/badge/Windows-0078D6?style=for-the-badge&logo=windows&logoColor=white" /></a>
  <a href="https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip" title="Download for MacOS"><img src="https://img.shields.io/badge/MacOS-c0c0c0?style=for-the-badge&logo=apple&logoColor=black" /></a>
  <a href="https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip" title="Download for Linux"><img src="https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black" /></a>
</kbd>
</p>

*Training Mode* can be used to manually drive the rover using your mouse or keyboard. This is helpful for familiarizing yourself with the simulator and its environment. Manual driving can also be used to record sensor data for telemetry purposes.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/sim_screenshot.webp" alt="" width="98%">
<br>
<sup> The simulator in Training Mode. (Credit: Udacity)</sup>
</p>

However, to use the simulator in *Autonomous Mode*, the rover will have to be programmed for autonomous action. This will require [Python 3](https://www.python.org/downloads/) as the programming language, along with the following [package dependencies](https://www.activestate.com/resources/quick-reads/python-dependencies-everything-you-need-to-know/):

* [Flask](http://flask.pocoo.org/)—A micro web framework for Python
* [Numpy](http://www.numpy.org/)—A numerical computing package for Python that supports arrays and matrices
* [Jupyter](http://jupyter.org/install.html)—A web-based interactive platform that combines live code, equations, visualizations, etc.
* [Socketio](https://pypi.python.org/pypi/python-socketio)—A JavaScript framework for real-time web applications
* [Eventlet](http://eventlet.net/)—A concurrent networking framework for Python
* [Matplotlib](https://matplotlib.org/users/installing.html)—A plotting framework for Python
* [OpenCV 2](http://opencv.org/)—Open-Source Computer Vision: a software framework aimed at real-time [computer vision](https://en.wikipedia.org/wiki/Computer_vision) 
* [Python Imaging Library (PIL)](http://www.pythonware.com/products/pil/)—For opening, manipulating, and saving different image file formats

Most of these dependencies can be resolved with [Anaconda](https://www.continuum.io/anaconda-overview), an open-source Python package manager aimed at data science. Its mini-version, [Miniconda](https://conda.io/miniconda.html), can also be used.

Alternatively, a pre-built virtual environment, such as the [RoboND-Python-Starterkit](https://github.com/ryan-keenan/RoboND-Python-Starterkit) can be used along with Anaconda or Miniconda.

### What Constitutes a Successful Operation?

Each time the simulator is launched in autonomous mode, the rover is positioned in a random orientation in the middle of the world (the mission area), and six rock samples are randomly scattered across it.

The primary metrics of interest are:

* Percentage of mission area (or world) mapped
* Mapping Fidelity (the accuracy with which the world is mapped)
* Number of samples located
* Number of samples collected
* Time taken to accomplish the above, i.e., to complete the mission

Udacity's minimum criteria is to map at least 40% of the mission area at 60% fidelity and locate at least one of the rock samples (not required to collect).

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/min_requirements.webp" alt="" width="75%">
<br>
<sup> Results of a mission run in the simulator. (Credit: Udacity)</sup>
</p>

In contrast, the objective of the NASA Sample Return Challenge is not only to locate samples of interest but also to collect them and return them to the starting point!

Our mission, should we choose to accept it, is to locate *and* collect *all* six samples and bring them back to the starting point while mapping at least 90% of world with over 75% fidelity in the least possible amount of time.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/rover-auto-trim-compress-v2.gif" alt="" width="75%">
<br>
<sup> The rover in autonomous mode. (Credit: Udacity)</sup>
</p>

In the best-case scenario, the entire environment is mapped at nearly 100% fidelity, but this may be beyond the scope of this project.

### Strategy for Developing the Rover's Vision

The rover should be able to see where to drive and where not to, e.g., rocks and mountains. It should also be able to identify the target samples of interest. Ultimately, it should be able to know where it is on a known map of the mission area, the *worldmap*, so it can navigate over it. And it must accomplish all of this using a front-mounted video camera. This is the primary sensor on the rover and reads color images of the environment in real-time. Note that a video is just a sequence of images. We use these images from the video feed to develop the rover's vision.

In these incoming images from the rover cam, only three regions concern us:

- the navigable regions, where the rover can drive—these make up most of the ground terrain,

- the non-navigable regions, or obstacles, where the rover cannot drive: rocks and mountains, and

- the target samples themselves, which are gold-ish-colored rocks.

These three regions of interest, or ROIs, are referred to in the code as *nav*, *obs*, and *rocks* for short.

Preliminary perception techniques are developed and tested on sensor data in the [Jupyter notebook](https://jupyter.org/) file `Rover_Project_Test_Notebook.ipynb`.

The final code for the rover's perception is implemented in the `perception.py` [module](https://csjob.medium.com/modules-in-programming-language-78f2a27544d0). See the `perception_step` [function](https://en.wikipedia.org/wiki/Function_(computer_programming)) for details.

To autonomously drive the rover, it should be able to "see" the area where it can drive (navigable area). The project assumes that the sand on the ground throughout the simulated mission area is lighter in color compared to the rest of the environment. We can then develop a driving strategy that involves, for starters, finding lighter-colored [pixels](https://en.wikipedia.org/wiki/Pixel) in the rover cam images to find navigable regions.

Pixels in an incoming camera image can be stored as an indexed collection of data, specifically a two-dimensional [array](https://en.wikipedia.org/wiki/Array_(data_structure)), where the indexes are the pixel locations and the data stored are the pixel colors. This allows us to manipulate the image by performing operations on the array.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/image-as-array.webp" alt="" width="55%">
<br>
<sup> Storing an image as an array. (Credit: Eye image: Paul Howson / tdgq.com.au)</sup>
</p>

Images from the rover cam have a dimension of 320x160 pixels. These can be stored as [NumPy](https://numpy.org/) arrays—of type 8-[bit](https://en.wikipedia.org/wiki/Bit) [unsigned integer](https://users.cs.utah.edu/~germain/PPS/Topics/unsigned_integer.html) (uint8)—defined as (160, 320, 3), meaning the image size is 160 pixels in the y-direction (height), 320 pixels in the x-direction (width), and it has three layers, or "color channels." These three color channels of the image are the primary additive colors of red, green, and blue, or "RGB." These can be mixed (per the [RGB color model](https://en.wikipedia.org/wiki/RGB_color_model)) to produce a broad array of colors. More specifically, the combination of intensity values from 0 to 255 across these three channels determines what color is seen in the image. Darker pixels have low intensity values, while brighter pixels have high values. The color black (darkest) has an RGB value of R = 0, G = 0, B = 0, or simply (0, 0, 0), while the color white (brightest) has an RGB value of (255, 255, 255).

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/color-channels-bd.webp" alt="" width="99%">
<br>
<sup> The same camera image is displayed separately on each of its red, green, and blue color channels.</sup>
</p>

Notice that while the mountains are relatively dark in all three color channels, both the ground and the sky are brighter, with the ground being the brightest. Since the different regions in the image all have distinct color intensities, it is possible to extract them individually from the image using a [color threshold](https://en.wikipedia.org/wiki/Thresholding_(image_processing)). For example, we can only extract those pixels from the image that correspond to the ground.

### Identifying ROIs Using Color Thresholding

A color threshold converts a color image to a black-and-white image, highlighting (in white) a single region of interest (ROI) whose color intensity falls above the threshold. 

In other words, it takes an input color image, discards all pixels from the image with intensity values below the threshold, and outputs a binary, black-and-white image with only those pixels whose intensity values fall above the threshold.

We need a color threshold that can distinguish between navigable (ground) and non-navigable terrain (rocks and mountains). From some trial and error, we find that above an RGB threshold of (160, 160, 160), only ground pixels can be seen.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/nav-threshed-bd.webp" alt="" width="99%">
<br>
<sup> Using a color threshold on a 3-channel image to extract only ground (navigable) terrain pixels into a "threshed," single-channel image.</sup>
</p>

Like the input color image, the output *threshed* image is also stored as an array. But while the color image has three channels (RGB), the threshed image has a single channel: visible or invisble, indicated by a value of one or zero. Hence, it is also called a binary image.

This is how the threshold is applied: we create a single-channel Numpy array of zeros for storing the threshed image. This output array is the same size as the input camera image array. This implies that the indexes of both arrays correspond to the same pixel locations. Then, for every camera image received, we traverse its array to read the RGB value of each pixel. Wherever the RGB value is *above* the selected threshold, we set the value in the corresponding position in the output single-channel array to one. This enables us to see only our desired ROI: the ground.

However, when it comes to identifying the rock samples, only measuring the RGB color intensity proves insufficient. We have to use a more descriptive model instead, with more color parameters, like the [HSV](https://en.wikipedia.org/wiki/HSL_and_HSV), which captures the [hue](https://en.wikipedia.org/wiki/Hue), color [saturation](https://en.wikipedia.org/wiki/Colorfulness), and brightness [value](https://en.wikipedia.org/wiki/Brightness) of pixels instead of just their color intensity.

The following HSV color range is used to capture the variation in hue, saturation, and brightness in the color of the rock samples:

- lower bound = (75, 130, 130)
- upper bound = (255, 255, 255)

Since the incoming rover cam images are in the RGB color format, they are converted to the HSV format and then *thresholded* on this range to only extract pixels identifying rock samples.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/hsv-threshed-bd.webp" alt="" width="98%">
<br>
<sup> Identification of gold-colored rocks by applying an HSV threshold.</sup>
</p>

Like the RGB threshold, only pixels with color properties within the HSV threshold range are extracted. 

### Target Region Found! But How Do We Get There?

Having identified an ROI, we now need to get to it. Specifically, we need to know where it is in relation to the rover. In which direction, or [heading](https://en.wikipedia.org/wiki/Heading_(navigation)), should the rover drive to reach that ROI? 

One approach is to find the angle between the rover and the ROI, which will give the required rover heading. But, geometrically speaking, we cannot find an angle between two lines that are on different planes. The rover's position is in relation to the two-dimensional ground, while the ROIs are in three-dimensional images. 

In short, we'll have to project the 3D camera images onto the 2D ground! This is where [perspective transformation](https://en.wikipedia.org/wiki/3D_projection#Perspective_projection) comes in. It is a type of 3D projection; it projects, or maps, a 3D image onto a 2D plane. Note that the terms *perspective* and *warped* are also used to refer to the *projected* image. All three are synonymous terms and are used interchangeably.

Depending on how we calibrate the perspective transformation, we can achieve a specific size for the projected image in 2D.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/perspect-transform-bd.webp" alt="" width="99%">
<br>
<sup> Calibrating a perspective transform where a 1 square-meter region in the 3D camera image corresponds to a 10x10 pixel square in its 2D ground projection.</sup>
</p>

We want to calibrate the transform such that a one-square-meter ground region in 3D projects to a 10x10 pixel square in 2D. This gives the rover a decent view of the ground ahead and plenty of time to react appropriately to different regions of interest.

<u>In more detail</u>: The 3D image above is specially taken from the rover cam of the sim environment overlayed with a grid of known size, where each grid cell is one square meter. To calibrate the transform, the edges of a grid cell in this image are taken as the four source [coordinates](https://en.wikipedia.org/wiki/Cartesian_coordinate_system) of the transformation. Then, four corresponding destination coordinates are chosen on the 2D plane such that they demarcate a 10x10 pixel square. These source and destination coordinates are fed into the OpenCV function `getPerspectiveTransform` to get the specific [transform matrix](https://en.wikipedia.org/wiki/Transformation_matrix) that represents this particular perspective transformation. The transform matrix is then fed into the OpenCV function `warpPerspective`  to accordingly project the rover cam images to the ground. See the Jupyter test notebook and the perception module in the code for details.

Once we have a 2D ground projection of the rover's 3D cam view, we can apply the color thresholds described previously to see the ground with a filtered view for each ROI.

<p align="left">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/color-thresh-io-bd.webp" alt="" width="99.9%">
<br>
<sub> The color_thresh function takes an input warped image and converts it into three separate single-channel images, identifying each of the three ROIs.</sub>
</p>
<!--In the `color_thresh` function, three different color thresholds are applied to incoming images from the rover camera to extract the (*threshed*) binary images corresponding to, and identify, each ROI.-->

The rover can now "see" what's ahead on the ground to scale, which means it can now compute and travel specific distances. But the rover's view is not yet oriented with respect to its own [coordinate system](https://en.wikipedia.org/wiki/Coordinate_system). You see, the pixels in the projected (or perspective) image are described with respect to the image's own coordinate reference frame. But the rover's coordinate frame is different; e.g., we've chosen to orient the rover's x-axis along the direction it points to.

So, we need to re-calculate the pixel positions from the rover's POV, i.e., with respect to the rover's coordinate system. This is done in the `perspect_to_rover` function.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/perspect-to-rover-bd.webp" alt="" width="95%">
<br>
<sup> Expressing the perspective view in the rover's coordinate frame (top-down view).</sup>
</p>

Notice that, after the recalculation, the projected/perspective image is centered at the origin of the rovers' coordinate frame. Fixing a coordinate system relative to the robot is central to many robotic applications as it allows objects in the robot's environment to be described relative to the robot; more specifically, relative to the robot's camera in this case.

### Finding Drive Heading and Navigating the Mission Area

The rover is now able to compute the heading and distance to the desired pixel coordinates. For example, it can calculate the heading and distance to a target sample. More fundamentally, it can compute the average angle between itself and all the navigable pixels (nav heading) to determine the direction in which it can drive.

The entire process of determining the nav heading from a front cam image is summarized below.

<p align="left">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/img-proc-pipe-bd.webp" alt="" width="55%">
<br>
<sub> Complete image processing pipeline for converting the rover's front-cam view to a ground projection of navigable terrain.</sub>
</p>
<!--<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/perception_step_1.png" alt="" width="99.9%">-->

The red arrow above indicates the nav heading, or nav angle, the average angle from the rover to the pixels of the navigable region.

The nav heading enables the rover to drive around without crashing into rocks and mountains. But we need a way to efficiently traverse the entire mission area, so we're not aimlessly driving around forever. This involves not revisiting an area unnecessarily that has already been searched and knowing when to stop searching.

As an alternative to [motion-planning](https://en.wikipedia.org/wiki/Motion_planning), which involves search algorithms for navigation, we're going to follow a simpler approach, consisting of *wall-following* and mapping ROI pixels to a known world map.

The idea is for the rover to always keep the mountains and large rocks—collectively, the *wall*—to either side: left *or* right. This ensures that it always visits an area once and doesn't lose its way. We can implement wall-following by adding an offset to the nav angle to bias the heading such that the rover always follows the left wall. 

There's a better approach, though. Instead of adding an offset to the nav angle, which is the average angle to *all* nav pixels, we can improve wall-following by instead adding an offset to the average angle to only *those* pixels that lie to the left of the rover (nav_angle_left).

<p align="left">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/wall-following-v2.webp" alt="" width="95%">
<br>
<sub> The wall-following approach. Recall that the rover heading (where the rover points) is defined by the x-axis of the Rover Coordinate Frame.
<br> • nav_angle is the average angle to all navigable terrain pixels. 
<br> • nav_angle_right is the average angle to nav pixels that lie to the right of the rover. 
<br> • nav_angle_left is the average angle to nav pixels that lie to the left of the rover. 
</sub>
</p>

While a biased rover heading obtained by offsetting the nav_angle will work in some scenarios, it will not work in those where the nav terrain has a significant deviation from the rover's current heading. It is this same scenario that is depicted in the illustration above. Note how the nav terrain is considerably deviated from the wall and how nav_angle_left is much smaller compared to nav_angle. Since nav_angle_left is comparatively a much better approximation of the wall angle from the rover, a biased rover heading obtained from using this angle as an offset will be able to account for the majority of wall-nav deviation scenarios, resulting in a more robust wall-following behavior.

The second part of navigating the ground is knowing when to finally stop searching. As mentioned, we will do this by tracking the rover's coverage of the world on a known map. This requires expressing the ROIs in the *world coordinate frame* so they can be superimposed onto this map, so we can determine how much of it is already covered and how much of it remains.

<!--~~The `FollowWall` class governs the actions taken in this state, while the `following_wall` function handles transitions from this state.~~-->

### Mapping Regions To the World Map

Ideally, the rover should cover (or map) as much of the world (the designated mission area) as possible so as not to miss any target samples. Furthermore, of the area it does cover, we need to know how much of it corresponds to the actual mission area (fidelity). This requires comparing the rover's actual coverage of the world to a (known) [ground truth](https://en.wikipedia.org/wiki/Ground_truth) map. 

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/ground_truth_map.webp" alt="" width="40%">
<br>
<sub> The ground truth map of the world covers a 0.2 square kilometer mission area. (Credit: Udacity)</sub>
</p>

To compare the actual coverage of the mission area with its ground truth, we have to map, or superimpose, ground projections of ROIs onto the ground truth map. This requires the ROI projections to be on the *same* coordinate frame as the map itself, i.e., the *world coordinate frame*. So, we have to transform ROI projections from the rover's coordinate frame to the world's coordinate frame.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/rover-to-world-bd.webp" alt="" width="98%">
<br>
<sup> Transforming the rover's coordinate frame to the world coordinate frame. (Credit: Udacity)</sup>
</p>

The *rover coordinate frame* can be [geometrically transformed](https://en.wikipedia.org/wiki/Geometric_transformation) to the *world coordinate frame* by (1) [rotating](https://en.wikipedia.org/wiki/Rotation_matrix) the frame with an angle equal to the rover's [yaw angle](https://en.wikipedia.org/wiki/Yaw_(rotation)) and (2) [translating](https://en.wikipedia.org/wiki/Translation_(geometry)) it with a displacement equal to the rover's position. This is done in the `rover_to_world` function in the perception module.

The rotation is required to account for the fact that when the rover's camera captures an image, the rover can be pointing in any arbitrary direction—its yaw angle. The translation accounts for the fact that the rover may be located at any position in the world when it captures an image. Hence, note that simply recalculating pixel locations—as previously done to convert from perspective image to rover coordinate frame—is not sufficient in this case.

Once all ROI pixels are available relative to the world coordinate frame, their RGB values are copied to their corresponding positions on the ground truth map. The ground truth map is stored as a three-channel array so that these color values can be overlayed on it.

This implies that ROIs can be highlighted on the map itself in real-time by updating this worldmap array with each incoming image from the rover cam.

<p align="left">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/test_mapping.gif" alt="" width="98%">
<br>
<sub> Mapping (or superimposing) ground projections of ROI pixels onto the ground truth map. <br>Color code: blue (navigable); red ( non-navigable); white (target rock samples).</sub>
</p>

Recognize that traversing the mission area with high fidelity depends on 2D ground projections that faithfully depict their source 3D camera images. But accurately projecting rover-cam images to the ground plane requires a stable camera, which is not always the case; the ground terrain, acceleration, deceleration, and steering all affect the rover's stability. A stable drive implies that the rover is not lifting or rolling off of the ground plane, as [characterized by](https://en.wikipedia.org/wiki/Degrees_of_freedom_(mechanics)) its pitch and roll angles.

Thus, to optimize mapping fidelity, the following steps are taken to improve the 3D to 2D conversion accuracy in perspective transforms:

- (a) ROI pixels beyond certain distances from the rover are trimmed out of rover-cam images.

- (b) Entire rover-cam images are discarded and not updated on the worldmap when the rover is not in stable drive, that is, when its pitch and roll angles are outside of certain bounds.

But this increased accuracy comes at the cost of the total area covered. In (a), we are directly omitting ROI pixels, and in (b), whole images from the camera feed are ignored altogether. When a rover-cam image is discarded, it does not get projected on the region of the ground where the rover was at the moment of taking that image. So, that region of the ground is never covered for any ROIs.

So, there has to be a compromise between fidelity and map coverage—one comes at the cost of the other. If one is too high, the other will be too low. We have chosen to optimize the two to ensure at least 90% map coverage (results to follow).

### Using a State Machine for Making Decisions

A [state machine](https://en.wikipedia.org/wiki/Finite-state_machine), or finite state machine (FSM), is an abstract machine representing a mathematical model of computation. It can be in exactly one of a finite number of states at any given time, performing some specific action. The state it is currently in is called the active state. External events can trigger transitions from one active state to another. When implemented in code, each state may also have an associated *handler* that supervises transitions from that state.

An FSM *listens* for external events, and it *transitions* to a state (of action) when triggered by an event it was listening for. The following set of *events*, *transitions*, and *action states* constitute the rover's decision-making capability.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/fsm-bd.webp" alt="" width="99%">
<br>
<sup> The rover's state machine represents its decision-making capability. Key: circles (states); labels (events); arrows (transitions).</sup>
</p>

The circles indicate a *state*, e.g., "Follow Wall," while arrows indicate *transitions* to other states triggered by external *events*. For example, "at_left_obstacle" triggers a transition to the "Avoid Wall" state.

The state machine for the rover's decision-making is implemented in the `decision_new.py` module, with assistance from `states.py`, `events.py`, and `handlers.py`. 

<!--This module implements the rover's state machine with the help of the following additional modules:

- `events.py` defines the set of events that trigger state transitions~~
- `handlers.py` determines the possible next states from the current active state~~
- `states.py` defines the state actions that are performed when a state is active-->

### Collecting Rock Samples, Getting Unstuck, and Returning Home

The strategy for locating and collecting rock samples is devised in such a way that it does not conflict with wall-following, where the rover would lose the wall—and its way—while going after a rock sample. To avoid such a situation, samples are only detected if they are nearby, so that the rover can easily find the wall again after collecting them. This is achieved by using a rover-to-rock distance limit beyond which rock samples are ignored. This still allows us to locate and retrieve all rock samples since if a sample is further away—and ignored—while wall-following in one direction, the same sample will be that much closer as the rover makes its way back to it while following the wall on the opposite side.  

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/rock-pickup2.gif" alt="" width="65%">
<br>
<sup> Retrieving the target rock sample. (Credit: Udacity)</sup>
</p>

<!--~~The `GoToSample` class governs the actions taken in this state while the `going_to_sample` function handles transitions from this state.~~-->

It is important to know that since the rover always turns left to seek the left wall, it can bump into it around certain corners and get stuck. The rover can also get stuck in between smaller rocks that enter its field of view too late. So, to detect if the rover has gotten stuck somewhere, a timer is started every time the velocity drops to zero. If the time exceeds the designated *stuck time* for a given state, the "Get Unstuck" state is activated, which spins the wheels faster to try to break the rover loose. If sufficient velocity is reached during these maneuvers, meaning the rover is driving again, the "getting_unstuck" handler switches to another state. The timer has to be switched off every time the rover transitions to a different state to account for the possibility of also getting stuck in that state.

As soon as the mission is completed—all samples collected and 95% of the area mapped—the "Return Home" state is triggered. The rover finds the *home* destination by expressing the home coordinates (given) from the world coordinate frame to its own reference frame. It then tracks the distance and heading to these coordinates by converting them from [cartesian](https://en.wikipedia.org/wiki/Cartesian_coordinate_system) to [polar](https://en.wikipedia.org/wiki/Polar_coordinate_system) format. For details, see functions `world_to_rover` and `to_polar_coords` in the perception module .


Recall that the *nav heading* is given by nav_angle_left; it is the heading the rover drives at while in the "Follow Wall" state. On the other hand, the *home heading* is the angle from the rover to the home coordinates.

The rover's remaining distance to home is used to vary its speed and heading profiles as follows:

- When the rover is sufficiently far away from home, a pure nav heading is used.

- When the rover gets closer to home, a [weighted average](https://en.wikipedia.org/wiki/Weighted_arithmetic_mean) of home and nav headings is used with a 3:7 ratio.

- When the rover is fairly close to home, a pure home heading is used with a more controlled speed.

<!--
~~The `ReturnHome` class governs the actions taken in this state, while the `returning_home` function handles transitions from this state.~~

~~==TODO:== Add here somewhere:~~

~~To be able to locate coordinates on the world, e.g., those of the rover's starting location, functions for inverse transformations are added to convert pixel points from the world frame to the rover's frame:~~

* ~~`inv_translate_pixpts`~~
* ~~`inv_rotate_pixpts`~~
* ~~`world_to_rover`~~

-->

### Taking the Rover on a Test Mission

The simulator's Autonomous Mode is launched from the command line with the following command:

```sh
python drive_rover.py
```

The mission is deemed complete when the rover has collected all six rock samples and mapped at least 95% of the environment, or time spent doing so has exceeded 680 seconds, whichever comes first.

If the rover gets stuck in a repetitive behavior, it takes anywhere from 2 to 5 seconds for the *unstuck* routine to kick in, depending on the circumstances leading to that behavior.

<p align="left"; style="line-height: 100% !important">
<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/demo_results.webp" alt="" width="99%">
<br>
<sup> The rover after successfully completing the mission.</sup>
</p>

After multiple runs in the simulator, the following results are achieved:

<table border="0">
 <tr>
    <td>Environment mapped</td>
    <td><i>92 - 96 %</i></td>
 </tr>
 <tr>
    <td>Mapping fidelity</td>
    <td><i>> 80%</i></td>
 </tr>
 <tr>
    <td>Rocks located</td>
    <td><i>≥ 6</i></td>
 </tr>
 <tr>
    <td>Rocks collected</td>
    <td><i>≥ 6</i></td>
 </tr>
 <tr>
    <td>Time Taken</td>
    <td><i>690 - 780 s</i></td>
 </tr>
</table>

**Note:** The simulator is run on a machine with the following specifications:

<table border="0">
 <tr>
    <td>Operating System</td>
    <td><i>ubuntu 16.04 LTS 64-bit</i></td>
 </tr>
 <tr>
    <td>Processor Type</td>
    <td><i>Intel® Core™ i7-7700</i></td>
 </tr>
 <tr>
    <td>Processor Speed</td>
    <td><i>3.60 GHz x 8</i></td>
 </tr>
 <tr>
    <td>Processor Cores</td>
    <td><i>4</i></td>
 </tr>
 <tr>
    <td>Processor Cache</td>
    <td><i>8 MB</i></td>
 </tr>
 <tr>
    <td>Graphics</td>
    <td><i>NVIDIA GeForce GTX 1050</i></td>
 </tr>
 <tr>
    <td>Video Memory</td>
    <td><i>2 GB</i></td>
 </tr>
</table>

**Note:** The *autonomous mode* has so far been tested with the following simulator settings:

<table>
 <tr>
    <td>Graphics Quality</td>
    <td><i>Fantastic</i></td>
 </tr>
 <tr>
    <td>Screen Resolution</td>
    <td><i>1440 x 960</i></td>
 </tr>
</table>

#### Room For Improvement?

In the current implementation, the rover is unable to negotiate more tricky obstacle scenarios, such as rocks protruding from the sides of the mountains. By doing further analysis of the rover-cam sensor data, additional computer vision techniques can be devised to address this issue.

Furthermore, to traverse the *world* more efficiently, a path-planning strategy can be used instead of wall-following, such as [Dijkstra's](https://en.wikipedia.org/wiki/Dijkstra's_algorithm) or [A-star](https://en.wikipedia.org/wiki/A*_search_algorithm) search algorithms.

A reimplementation in [C++](https://medium.com/nerd-for-tech/why-c-is-the-best-programming-language-2385b60d7e25) can also be considered for [faster program execution](https://towardsdatascience.com/how-fast-is-c-compared-to-python-978f18f474c7) to get a snappier response from the rover.

------------
> Copyright © 2017-2024, Salman Hashmi.

[![License: BSD](https://img.shields.io/badge/License-BSD-brightgreen.svg)](https://github.com/Salman-H/mars-search-robot?tab=License-1-ov-file#readme) 
