"""
Module for rover perception.

Contains functions for processing rover's front camera video frames
and updating rover state

"""

__author__ = 'Salman Hashmi, Ryan Keenan'
__license__ = 'BSD License'


import numpy as np
import cv2


# CONSTANTS

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


def to_rover_coords(binary_img):
    """Convert all points on img coord-frame to those on rover's frame."""
    # Identify nonzero pixels in binary image representing
    # region of interest e.g. rocks
    ypos, xpos = binary_img.nonzero()

    # Calculate pixel positions with reference to rover's coordinate
    # frame given that rover front cam itself is at center bottom of
    # the photographed image.
    xpix = -(ypos - binary_img.shape[0]).astype(np.float)
    ypix = -(xpos - binary_img.shape[1]/2).astype(np.float)
    return xpix, ypix


def to_polar_coords(xpix, ypix):
    """Convert cartesian coordinates to polar coordinates."""
    # compute distance and angle of 'each' pixel from origin and
    # vertical respectively
    distances = np.sqrt(xpix**2 + ypix**2)
    angles = np.arctan2(ypix, xpix)
    return distances, angles


def rotate_pix(xpix, ypix, angle):
    """Apply a geometric rotation."""
    angle_rad = angle * np.pi / 180  # yaw to radians
    xpix_rotated = (xpix * np.cos(angle_rad)) - (ypix * np.sin(angle_rad))
    ypix_rotated = (xpix * np.sin(angle_rad)) + (ypix * np.cos(angle_rad))
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    """Apply a geometric translation and scaling."""
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    return xpix_translated, ypix_translated


def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    """
    Apply a geometric transformation i.e. rotation and translation to ROI.

    Keyword arguments:
    xpix, ypix -- numpy array coords of ROI being converted to world frame
    xpos, ypos, yaw -- rover position and yaw angle in world frame
    world_size -- integer length of the square world map (200 x 200 pixels)
    scale -- scale factor between world frame pixels and rover frame pixels

    Note:
    Requires functions rotate_pix and translate_pix to work

    """
    # Apply rotation and translation
    xpix_rot, ypix_rot = rotate_pix(
        xpix, ypix, yaw
    )
    xpix_tran, ypix_tran = translate_pix(
        xpix_rot, ypix_rot, xpos, ypos, scale
    )
    # Clip pixels to be within world_size
    xpix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    ypix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)

    return xpix_world, ypix_world


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


def perception_step(Rover):
    """Apply above functions to update Rover state accordingly."""
    # Apply perspective transform
    warped_img = perspect_transform(Rover.img)
    # Apply color threshold to identify
    # navigable terrain/obstacles/rock samples
    thresh_img_nav, thresh_img_obs, thresh_img_rock = color_thresh(warped_img)

    # Update Rover.vision_image
    # (this will be displayed on left side of screen)
    Rover.vision_image[:, :, 0] = threshed_img_obstacle*135
    Rover.vision_image[:, :, 1] = threshed_img_rock
    Rover.vision_image[:, :, 2] = threshed_img_navigable*175

    # Convert map image pixel values to rover-centric coords
    nav_x_rover, nav_y_rover = to_rover_coords(thresh_img_nav)
    obs_x_rover, obs_y_rover = to_rover_coords(thresh_img_obs)
    rock_x_rover, rock_y_rover = to_rover_coords(thresh_img_rock)

    # Convert rover-centric navigable pixel positions to polar coordinates
    Rover.nav_dists = to_polar_coords(nav_x_rover, nav_y_rover)[0]
    Rover.nav_angles = to_polar_coords(nav_x_rover, nav_y_rover)[1]

    # Convert rover-centric obstacle pixel positions to polar coordinates
    Rover.obs_dists = to_polar_coords(obs_x_rover, obs_y_rover)[0]
    Rover.obs_angles = to_polar_coords(obs_x_rover, obs_y_rover)[1]

    # Convert rover-centric rock sample pixel positions to polar coordinates
    Rover.rock_dists = to_polar_coords(rock_x_rover, rock_y_rover)[0]
    Rover.rock_angles = to_polar_coords(rock_x_rover, rock_y_rover)[1]

    # Convert rover-centric pixel values to world coordinates
    nav_x_world, nav_y_world = pix_to_world(
        nav_x_rover, nav_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw,
        Rover.worldmap.shape[0], SCALE_FACTOR
    )
    obs_x_world, obs_y_world = pix_to_world(
        obs_x_rover, obs_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw,
        Rover.worldmap.shape[0], SCALE_FACTOR
    )
    rock_x_world, rock_y_world = pix_to_world(
        rock_x_rover, rock_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw,
        Rover.worldmap.shape[0], SCALE_FACTOR
    )
    # Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 255
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 255
    Rover.worldmap[nav_y_world, nav_x_world, 2] += 255

    return Rover
