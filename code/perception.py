"""
Module for rover perception.

Contains functions for processing rover's front camera video frames
and updating rover state

"""

__author__ = 'Salman Hashmi, Ryan Keenan'
__license__ = 'BSD License'


from collections import namedtuple

import numpy as np
import cv2

from constants import TO_DEG, TO_RAD


def color_thresh(input_img, rgb_thresh=(160, 160, 160),
                 low_bound=(75, 130, 130), upp_bound=(255, 255, 255)):
    """
    Apply color thresholds to extract pixels of navigable/obstacles/rocks.

    Keyword arguments:
    input_img -- numpy image on which RGB threshold is applied
    rgb_thresh -- RGB thresh tuple above which only ground pixels are detected
    low/up_bounds -- HSV tuples defining color range of gold rock samples

    Return value:
    thresh_imgs -- namedtuple of binary images identifying nav/obs/rock pixels

    """
    # Create arrays of zeros same xy size as input_img, but single channel
    nav_img = np.zeros_like(input_img[:, :, 0])
    obs_img = np.zeros_like(input_img[:, :, 0])

    # Convert BGR input_img to HSV for rock samples
    hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

    # Require that each of the R(0), G(1), B(2) pixels be above all three
    # rgb_thresh values such that pixpts_above_thresh will now contain a
    # boolean array with "True" where threshold was met
    pixpts_above_thresh = ((input_img[:, :, 0] > rgb_thresh[0])
                           & (input_img[:, :, 1] > rgb_thresh[1])
                           & (input_img[:, :, 2] > rgb_thresh[2]))

    pixpts_nonzero = ((input_img[:, :, 0] > 0)
                      & (input_img[:, :, 1] > 0)
                      & (input_img[:, :, 2] > 0))

    # obstacle pixels are those non-zero pixels where rgb_thresh was not met
    obs_pixpts = np.logical_and(
        pixpts_nonzero, np.logical_not(pixpts_above_thresh)
        )
    # Index the array of zeros with the boolean array and set to 1
    # those pixels where ROI threshold was met
    nav_img[pixpts_above_thresh] = 1
    obs_img[obs_pixpts] = 1

    # Threshold the HSV image to get only colors for gold rock samples
    rock_img = cv2.inRange(hsv_img, low_bound, upp_bound)

    # Return the threshed binary images
    ThreshedImages = namedtuple('ThreshedImages', 'nav obs rock')
    thresh_imgs = ThreshedImages(nav_img, obs_img, rock_img)

    return thresh_imgs


def perspect_transform(src_img, dst_grid=10, bottom_offset=6):
    """
    Apply a perspective transformation to input 3D image.

    Keyword arguments:
    src_img -- 3D numpy image on which perspective transform is applied
    dst_grid -- size of 2D output image box of 10x10 pixels equaling 1 Sq m
    bottom_offset -- bottom of cam image is some distance in front of rover

    Return value:
    dst_img -- 2D warped numpy image with overhead view

    """
    # Dimension of source image from rover camera
    height, width = src_img.shape

    # Numpy array of four source points defining a grid on input 3D image
    # acquired from calibration data in test notebook
    src_x1, src_y1 = 14, 140
    src_x2, src_y2 = 301, 140
    src_x3, src_y3 = 200, 96
    src_x4, src_y4 = 118, 96

    # Corresponding destination points on output 2D overhead image
    dst_x1, dst_y1 = (width/2 - dst_grid/2), (height-bottom_offset)
    dst_x2, dst_y2 = (width/2 + dst_grid/2), (height-bottom_offset)
    dst_x3, dst_y3 = (width/2 + dst_grid/2), (height-dst_grid-bottom_offset)
    dst_x4, dst_y4 = (width/2 - dst_grid/2), (height-dst_grid-bottom_offset)

    src_points_3d = np.float32([[src_x1, src_y1],
                                [src_x2, src_y2],
                                [src_x3, src_y3],
                                [src_x4, src_y4]])

    dst_points_2d = np.float32([[dst_x1, dst_y1],
                                [dst_x2, dst_y2],
                                [dst_x3, dst_y3],
                                [dst_x4, dst_y4]])

    transform_matrix = cv2.getPerspectiveTransform(src_points_3d,
                                                   dst_points_2d)
    # Keep same size as source image
    dst_img = cv2.warpPerspective(src_img, transform_matrix, (width, height))

    return dst_img


def perspect_to_rover(binary_img):
    """
    Transform pixel points from perspective frame to rover frame.

    Keyword arguments:
    binary_img -- single channel 2D warped numpy image in perspective frame

    Return value:
    pixpts_rf -- namedtuple of numpy arrays of pixel x,y points in rover frame

    """
    # Dimension of input image
    height, width = binary_img.shape

    # Identify all nonzero pixel coords in the binary image
    ypix_pts_pf, xpix_pts_pf = binary_img.nonzero()

    # Calculate pixel positions with reference to rover's coordinate
    # frame given that rover front camera itself is at center bottom
    # of the photographed image
    xpix_pts_rf = -(ypix_pts_pf - height).astype(np.float)
    ypix_pts_rf = -(xpix_pts_pf - width/2).astype(np.float)

    # Define a named tuple for the three regions of interest
    PixPointsRf = namedtuple('PixPointsRf', 'x y')
    pixpts_rf = PixPointsRf(xpix_pts_rf, ypix_pts_rf)

    return pixpts_rf


def to_polar_coords(pixpts):
    """Convert cartesian coordinates of pixels to polar coordinates."""
    # Compute distances and angles of each pixel from rover
    dists_to_pixpts = np.sqrt(pixpts.x**2 + pixpts.y**2)
    angles_to_pixpts = np.arctan2(pixpts.y, pixpts.x)

    return dists_to_pixpts, angles_to_pixpts


def rotate_pixpts(pixpts, angle):
    """
    Geometrically rotate pixel points by specified angle.

    Keyword arguments:
    pixpts -- tuple of numpy arrays of pixel x,y points
    angle -- rotation angle

    Return value:
    pixpts_rot -- namedtuple of numpy arrays of pixel x,y points rotated

    """
    angle = angle*TO_RAD
    xpix_pts, ypix_pts = pixpts

    xpix_pts_rotated = xpix_pts*np.cos(angle) - ypix_pts*np.sin(angle)
    ypix_pts_rotated = xpix_pts*np.sin(angle) + ypix_pts*np.cos(angle)

    PixPointsRot = namedtuple('PixPointsRot', 'x y')
    pixpts_rot = PixPointsRot(xpix_pts_rotated, ypix_pts_rotated)

    return pixpts_rot


def translate_pixpts(pixpts_rot, rover_pos, scale_factor=10):
    """
    Geometrically translate rotated pixel points by rover position.

    Keyword arguments:
    pixpts_rot -- namedtuple of numpy arrays of pixel x,y points rotated
    rover_pos -- tuple of rover x,y position in world frame
    scale_factor -- between world and rover frame pixels

    Return value:
    pixpts_tran -- namedtuple of numpy arrays of pixel x,y points translated

    """
    rover_x, rover_y = rover_pos

    xpix_pts_translated = pixpts_rot.x/scale_factor + rover_x
    ypix_pts_translated = pixpts_rot.y/scale_factor + rover_y

    PixPointsTran = namedtuple('PixPointsTran', 'x y')
    pixpts_tran = PixPointsTran(xpix_pts_translated, ypix_pts_translated)

    return pixpts_tran


def rover_to_world(pixpts_rf, rover_pos, rover_yaw, world_size=200):
    """
    Transform pixel points of ROIs from rover frame to world frame.

    Keyword arguments:
    pixpts_rf -- tuple of numpy arrays of x,y pixel points in rover frame
    rover_pos -- tuple of rover x,y position in world frame
    rover_yaw -- rover yaw angle in world frame
    world_size -- integer length of square world map of 200 x 200 pixels

    Return value:
    pixpts_wf -- namedtuple of numpy arrays of pixel x,y points in world frame

    """
    # Apply rotation and translation
    pixpts_rot = rotate_pixpts(pixpts_rf, rover_yaw)
    pixpts_tran = translate_pixpts(pixpts_rot, rover_pos)

    # Clip pixels to be within world size
    xpix_pts_wf = np.clip(np.int_(pixpts_tran.x), 0, world_size-1)
    ypix_pts_wf = np.clip(np.int_(pixpts_tran.y), 0, world_size-1)

    # Define a named tuple for the points of the three ROIs
    PixPointsWf = namedtuple('PixPointsWf', 'x y')
    pixpts_wf = PixPointsWf(xpix_pts_wf, ypix_pts_wf)

    return pixpts_wf


def inv_translate_pixpts(pixpts_wf, translation, scale_factor=10):
    """
    Inverse translate pixel points from world frame.

    Keyword arguments:
    pixpts_wf -- namedtuple of numpy arrays of x,y pixel points in world frame
    translation -- tuple of displacements along x,y in world frame
    scale_factor -- between world and rover frame pixels

    Return value:
    pixpts_rot -- namedtuple of numpy arrays of pixel x,y points in prior
                  rotated positions
    """
    translation_x, translation_y = translation

    xpix_pts_rotated = (pixpts_wf.x - translation_x)*scale_factor
    ypix_pts_rotated = (pixpts_wf.y - translation_y)*scale_factor

    PixPointsRot = namedtuple('PixPointsRot', 'x y')
    pixpts_rot = PixPointsRot(xpix_pts_rotated, ypix_pts_rotated)

    return pixpts_rot


def inv_rotate_pixpts(pixpts_rot, angle):
    """
    Inverse rotate rotated pixel points to their original positions.

    Keyword arguments:
    pixpts_rot -- namedtuple of numpy arrays of x,y pixel points rotated
    angle -- rotation angle in degrees

    Return value:
    pixpts -- namedtuple of numpy arrays of pixel x,y points in
              original positions
    """
    angle = angle*TO_RAD

    xpix_pts = pixpts_rot.x*np.cos(angle) + pixpts_rot.y*np.sin(angle)
    ypix_pts = -pixpts_rot.x*np.sin(angle) + pixpts_rot.y*np.cos(angle)

    PixPoints = namedtuple('PixPoints', 'x y')
    pixpts = PixPoints(xpix_pts, ypix_pts)

    return pixpts


def world_to_rover(pixpts_wf, rover_pos, rover_yaw):
    """
    Transform pixel points of ROIs from world frame to rover frame.

    Keyword arguments:
    pixpts_wf -- tuple of numpy arrays of x,y pixel points in world frame
    rover_pos -- tuple of rover x,y position in world frame
    rover_yaw -- rover yaw angle in world frame

    Return value:
    pixpts_rf -- namedtuple of numpy arrays of pixel x,y points in rover frame

    """
    # Apply inverse translation and rotation
    pixpts_rot = inv_translate_pixpts(pixpts_wf, rover_pos)
    pixpts_rf = inv_rotate_pixpts(pixpts_rot, rover_yaw)

    return pixpts_rf


def perception_step(Rover, R=0, G=1, B=2):
    """
    Sense environment with rover camera and update rover state accordingly.

    Turns rover camera 3D images into a 2D perspective world-view of the
    rover environment identifying regions of interests and superimposes
    this view on the ground truth worldmap

    """
    # Apply perspective transform to get 2D overhead view of rover cam
    warped_img = perspect_transform(Rover.img)

    # Apply color thresholds to extract pixels of navigable/obstacles/rocks
    thresh_pixpts_pf = color_thresh(warped_img)

    # Update rover vision image with each ROI assigned to one of
    # the RGB color channels (displayed on left side of sim screen)
    R_VAL, G_VAL, B_VAL = 135, 1, 175
    Rover.vision_image[:, :, R] = thresh_pixpts_pf.obs * R_VAL
    Rover.vision_image[:, :, G] = thresh_pixpts_pf.rock * G_VAL
    Rover.vision_image[:, :, B] = thresh_pixpts_pf.nav * B_VAL

    # Transform pixel coordinates from perspective frame to rover frame
    nav_pixpts_rf = perspect_to_rover(thresh_pixpts_pf.nav)
    obs_pixpts_rf = perspect_to_rover(thresh_pixpts_pf.obs)
    rock_pixpts_rf = perspect_to_rover(thresh_pixpts_pf.rock)

    # Convert above cartesian coordinates to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(nav_pixpts_rf)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(obs_pixpts_rf)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_pixpts_rf)
    # Extract subset of nav_angles that are left of rover heading
    Rover.nav_angles_left = Rover.nav_angles[Rover.nav_angles > 0]

    # Only include pixels within certain distances from rover (for fidelity)
    nav_pixpts_rf = [pts[Rover.nav_dists < 60] for pts in nav_pixpts_rf]
    obs_pixpts_rf = [pts[Rover.obs_dists < 80] for pts in obs_pixpts_rf]
    rock_pixpts_rf = [pts[Rover.rock_dists < 70] for pts in rock_pixpts_rf]

    # Transform pixel points of ROIs from rover frame to world frame
    nav_pixpts_wf = rover_to_world(nav_pixpts_rf, Rover.pos, Rover.yaw)
    obs_pixpts_wf = rover_to_world(obs_pixpts_rf, Rover.pos, Rover.yaw)
    rock_pixpts_wf = rover_to_world(rock_pixpts_rf, Rover.pos, Rover.yaw)

    # Update Rover worldmap (to be displayed on right side of screen)
    MAX_RGB_VAL = 255
    Rover.worldmap[obs_pixpts_wf.y, obs_pixpts_wf.x, R] += MAX_RGB_VAL
    Rover.worldmap[rock_pixpts_wf.y, rock_pixpts_wf.x, G] += MAX_RGB_VAL
    Rover.worldmap[nav_pixpts_wf.y, nav_pixpts_wf.x, B] += MAX_RGB_VAL

    return Rover
