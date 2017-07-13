"""
Module for rover perception.

Contains functions for processing rover's front camera video frames
and updating rover state

Author: Salman Hashmi

"""

import numpy as np
import cv2


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


def rover_coords(binary_img):
    """Convert from image coords to rover coords."""
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position
    # being at the center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    """Convert to radial coords in rover space."""
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


def rotate_pix(xpix, ypix, yaw):
    """Map rover space pixels to world space."""
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    """Apply a scaling and a translation."""
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    """
    Apply rotation and translation (and clipping).

    Note:
    Requires functions rotate_pix and translate_pix to work

    """
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


def perspect_transform(img, src, dst):
    """Perform a perspective transform."""
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(
        img, M,
        (img.shape[1], img.shape[0]))  # keep same size as input image
    return warped


def perception_step(Rover):
    """Apply above functions to update Rover state accordingly."""
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify
    #    navigable terrain/obstacles/rock samples

    # 4) Update Rover.vision_image
    #    (this will be displayed on left side of screen)
    # Example:
    # Rover.vision_image[:,:,0] =
    #           obstacle color-thresholded binary image
    # Rover.vision_image[:,:,1] =
    #           rock_sample color-thresholded binary image
    # Rover.vision_image[:,:,2] =
    #           navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example:
    #   Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #   Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #   Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles
    return Rover
