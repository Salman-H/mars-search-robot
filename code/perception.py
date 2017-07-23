"""
Module for rover perception.

Contains functions for processing rover's front camera video frames
and updating rover state

"""

__author__ = 'Salman Hashmi, Ryan Keenan'
__license__ = 'BSD License'


import numpy as np
import cv2


# GLOBAL CONSTANTS

# rover cam image dimensions
CAM_IMG_WIDTH = 320
CAM_IMG_HEIGT = 160

# destination warped image box where 10x10 pixel square is 1 square meter
DST_GRID_SIZE = 10

# estimated bottom offset to account for bottom of cam image not being
# the position of the rover but a bit in front of it
BOTTOM_OFFSET = 6

# numpy array of four source coordinates on rover camera input 3D image
SRC_POINTS_3D = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])

# corresponding destination coordinates on output 2D overhead image
DST_POINTS_2D = np.float32([[CAM_IMG_WIDTH/2 - DST_GRID_SIZE/2,
                             CAM_IMG_HEIGT - BOTTOM_OFFSET],
                            [CAM_IMG_WIDTH/2 + DST_GRID_SIZE/2,
                             CAM_IMG_HEIGT - BOTTOM_OFFSET],
                            [CAM_IMG_WIDTH/2 + DST_GRID_SIZE/2,
                             CAM_IMG_HEIGT - DST_GRID_SIZE - BOTTOM_OFFSET],
                            [CAM_IMG_WIDTH/2 - DST_GRID_SIZE/2,
                             CAM_IMG_HEIGT - DST_GRID_SIZE - BOTTOM_OFFSET]])

# scale factor between world frame pixels and rover frame pixels
SCALE_FACTOR = 10

# integer length of the square world map (200 x 200 pixels)
WORLDMAP_HEIGHT = 200


def color_thresh(input_img, rgb_thresh=(160, 160, 160),
                 low_bound=(75, 130, 130), upp_bound=(255, 255, 255)):
    """
    Apply color thresholds to input image for extracting ROIs.

    Keyword arguments:
    input_img -- numpy image on which RGB threshold is applied
    rgb_thresh -- RGB thresh tuple above which only ground pixels are detected
    low_bound -- HSV lower bound tuple for color range of gold rock samples
    upp_bound -- HSV upper bound tuple for color range of gold rock samples

    Return values:
    nav_img -- binary image identifying ground/navigable terrain pixels
    obs_img -- binary image identifying rocks/obstacle terrain pixels
    rock_img -- binary image identifying rock sample terrain pixels

    """
    # Create arrays of zeros same xy size as input_img, but single channel
    nav_img = np.zeros_like(input_img[:, :, 0])
    obs_img = np.zeros_like(input_img[:, :, 0])

    # Convert BGR input_img to HSV for rock samples
    hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

    # Require that each of the R(0), G(1), B(2) pixels be above all three
    # rgb_thresh values such that pix_above_thresh will now contain a
    # boolean array with "True" where threshold was met
    pix_above_thresh = (
        (input_img[:, :, 0] > rgb_thresh[0]) &
        (input_img[:, :, 1] > rgb_thresh[1]) &
        (input_img[:, :, 2] > rgb_thresh[2])
    )
    pix_nonzero = (
        (input_img[:, :, 0] > 0) &
        (input_img[:, :, 1] > 0) &
        (input_img[:, :, 2] > 0)
    )
    # obstacle pixels are those non-zero pixels where rgb_thresh was not met
    obs_pix = np.logical_and(pix_nonzero, np.logical_not(pix_above_thresh))

    # Index the array of zeros with the boolean array and set to 1
    # those pixels where ROI threshold was met
    nav_img[pix_above_thresh] = 1
    obs_img[obs_pix] = 1

    # Threshold the HSV image to get only colors for gold rock samples
    rock_img = cv2.inRange(hsv_img, low_bound, upp_bound)

    # Return the binary image
    return nav_img, obs_img, rock_img


def perspect_transform(input_img):
    """
    Apply a perspective transformation to input 3D image.

    Keyword arguments:
    input_img -- 3D numpy image on which perspective transform is applied

    Return value:
    output_img -- 2D numpy image with overhead view

    """
    transform_matrix = cv2.getPerspectiveTransform(
        SRC_POINTS_3D,
        DST_POINTS_2D
    )
    output_img = cv2.warpPerspective(
        input_img,
        transform_matrix,
        (input_img.shape[1], input_img.shape[0])  # keep same size as input_img
    )
    return output_img


def perspect_to_rover(binary_img):
    """Transform pixel coords from perspective frame to rover frame."""
    # get image dimensions
    IMG_HEIGHT, IMG_WIDTH = binary_img.shape

    # Identify all nonzero pixel coords in the binary image
    ypixs_pf, xpixs_pf = binary_img.nonzero()

    # Calculate pixel positions with reference to rover's coordinate
    # frame given that rover front camera itself is at center bottom
    # of the photographed image
    xpixs_rf = -(ypixs_pf - IMG_HEIGHT).astype(np.float)
    ypixs_rf = -(xpixs_pf - IMG_WIDTH/2).astype(np.float)
    return xpixs_rf, ypixs_rf


def to_polar_coords(cartesian_coords):
    """Convert cartesian coordinates to polar coordinates."""
    # compute distance and angle of 'each' pixel from cartesian origin
    # and vertical respectively
    xpixs, ypixs = cartesian_coords
    distances = np.sqrt(xpixs**2 + ypixs**2)
    angles = np.arctan2(ypixs, xpixs)
    return distances, angles


def rotate_pixs(xpixs, ypixs, angle):
    """Apply a geometric rotation to pixel coords."""
    angle_rad = angle * np.pi / 180  # degrees to radians
    xpixs_rotated = (xpixs * np.cos(angle_rad)) - (ypixs * np.sin(angle_rad))
    ypixs_rotated = (xpixs * np.sin(angle_rad)) + (ypixs * np.cos(angle_rad))
    return xpixs_rotated, ypixs_rotated


def translate_pixs(xpixs_rot, ypixs_rot, rover_pos):
    """Apply a geometric translation and scaling to pixel coords."""
    rover_x, rover_y = rover_pos
    xpixs_translated = (xpixs_rot / SCALE_FACTOR) + rover_x
    ypixs_translated = (ypixs_rot / SCALE_FACTOR) + rover_y
    return xpixs_translated, ypixs_translated


def rover_to_world(xpixs_rf, ypixs_rf, rover_pos, rover_yaw):
    """
    Transform pixel coords of ROIs from rover frame to world frame.

    Keyword arguments:
    xpixs_rf -- numpy array of x coords of ROI pixels expressed in rover frame
    ypixs_rf -- numpy array of y coords of ROI pixels expressed in rover frame
    rover_pos -- rover cartesian (x,y) position in world frame
    rover_yaw -- rover yaw angle in world frame

    Return values:
    xpixs_wf -- numpy array of x coords of ROI pixels expressed in world frame
    xpixs_wf -- numpy array of x coords of ROI pixels expressed in world frame

    """
    # Apply rotation and translation
    xpixs_rf_rot, ypixs_rf_rot = rotate_pixs(xpixs_rf, ypixs_rf, rover_yaw)

    xpixs_rf_tran, ypixs_rf_tran = translate_pixs(xpixs_rf_rot,
                                                  ypixs_rf_rot, rover_pos)
    # Clip pixels to be within world_size
    xpixs_wf = np.clip(np.int_(xpixs_rf_tran), 0, WORLDMAP_HEIGHT - 1)
    ypixs_wf = np.clip(np.int_(ypixs_rf_tran), 0, WORLDMAP_HEIGHT - 1)

    return xpixs_wf, ypixs_wf


def perception_step(Rover):
    """Apply above functions to update Rover state accordingly."""
    # Apply perspective transform
    warped_img = perspect_transform(Rover.img)
    # Apply color threshold to identify
    # navigable terrain/obstacles/rock samples
    threshed_img_nav, threshed_img_obs, threshed_img_rock = color_thresh(
                                                                warped_img
                                                                )
    # Update Rover.vision_image
    # (this will be displayed on left side of screen)
    Rover.vision_image[:, :, 0] = threshed_img_obs*135
    Rover.vision_image[:, :, 1] = threshed_img_rock
    Rover.vision_image[:, :, 2] = threshed_img_nav*175

    # Transform pixel coords of ROIs (nav/obs/rocks) from
    # image perspective frame to rover frame
    nav_xpixs_rf, nav_ypixs_rf = perspect_to_rover(thresh_img_nav)
    obs_xpixs_rf, obs_ypixs_rf = perspect_to_rover(thresh_img_obs)
    rock_xpixs_rf, rock_ypixs_rf = perspect_to_rover(thresh_img_rock)

    # Convert above cartesian coords to polar coords
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(nav_xpixs_rf,
                                                        nav_ypixs_rf)

    Rover.obs_dists, Rover.obs_angles = to_polar_coords(obs_xpixs_rf,
                                                        obs_ypixs_rf)

    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_xpixs_rf,
                                                          rock_ypixs_rf)
    # Extract subset of nav_angles that are left of rover heading
    Rover.nav_angles_left = Rover.nav_angles[Rover.nav_angles > 0]

    # Discard distant pixel coords of ROIs to improve fidelity
    MAX_NAV_DISTS, MAX_OBS_DISTS, MAX_ROCK_DISTS = 60, 80, 70  # in meters

    # Only include pixel coords of a ROI that is in range
    nav_xpixs_rf = nav_xpixs_rf[Rover.nav_dists < MAX_NAV_DISTS]
    nav_ypixs_rf = nav_ypixs_rf[Rover.nav_dists < MAX_NAV_DISTS]

    obs_xpixs_rf = obs_xpixs_rf[Rover.obs_dists < MAX_OBS_DISTS]
    obs_ypixs_rf = obs_ypixs_rf[Rover.obs_dists < MAX_OBS_DISTS]

    rock_xpixs_rf = rock_xpixs_rf[Rover.rock_dists < MAX_ROCK_DISTS]
    rock_ypixs_rf = rock_ypixs_rf[Rover.rock_dists < MAX_ROCK_DISTS]

    # Transform pixel coords of ROIs (nav/obs/rocks) from
    # rover frame to world frame
    nav_xpixs_wf, nav_ypixs_wf = rover_to_world(nav_xpixs_rf,
                                                nav_ypixs_rf,
                                                Rover.pos, Rover.yaw)

    obs_xpixs_wf, obs_ypixs_wf = rover_to_world(obs_xpixs_rf,
                                                obs_ypixs_rf,
                                                Rover.pos, Rover.yaw)

    rock_xpixs_wf, rock_ypixs_wf = rover_to_world(rock_xpixs_rf,
                                                  rock_ypixs_rf,
                                                  Rover.pos, Rover.yaw)
    # Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obs_ypixs_wf, obs_xpixs_wf, 0] += 255
    Rover.worldmap[rock_ypixs_wf, rock_xpixs_wf, 1] += 255
    Rover.worldmap[nav_ypixs_wf, nav_xpixs_wf, 2] += 255

    return Rover
