"""
Helper functions for drive_rover module.

Used to update rover state as well as rover vision
and worldmap images on simulation screen

"""

__author__ = 'Salman Hashmi, Ryan Keenan'
__license__ = 'BSD License'


import time
import base64
from io import BytesIO, StringIO

import cv2
import numpy as np
from PIL import Image


def convert_to_float(string_to_convert):
    """
    Convert telemetry strings to float.

    This is done independent of decimal convention

    """
    if ',' in string_to_convert:
        float_value = np.float(string_to_convert.replace(',', '.'))
    else:
        float_value = np.float(string_to_convert)
    return float_value


def update_rover(Rover, data):
    """Update rover state."""
    # Initialize start time and sample positions
    if Rover.start_time is None:
        Rover.start_time = time.time()
        Rover.total_time = 0
        samples_xpos = np.int_(
              [convert_to_float(pos.strip())
               for pos in data["samples_x"].split(';')]
        )
        samples_ypos = np.int_(
              [convert_to_float(pos.strip())
               for pos in data["samples_y"].split(';')]
        )
        Rover.samples_pos = (samples_xpos, samples_ypos)
        Rover.samples_to_find = np.int(data["sample_count"])

    # Or just update elapsed time
    else:
        tot_time = time.time() - Rover.start_time
        if np.isfinite(tot_time):
            Rover.total_time = tot_time

    # Print out the fields in the telemetry data dictionary
    print(data.keys())
    # The current speed of the rover in m/s
    Rover.vel = convert_to_float(data["speed"])
    # The current position of the rover
    Rover.pos = [convert_to_float(pos.strip())
                 for pos in data["position"].split(';')]
    # The current yaw angle of the rover
    Rover.yaw = convert_to_float(data["yaw"])
    # The current yaw angle of the rover
    Rover.pitch = convert_to_float(data["pitch"])
    # The current yaw angle of the rover
    Rover.roll = convert_to_float(data["roll"])
    # The current throttle setting
    Rover.throttle = convert_to_float(data["throttle"])
    # The current steering angle
    Rover.steer = convert_to_float(data["steering_angle"])
    # Near sample flag
    Rover.near_sample = np.int(data["near_sample"])
    # Picking up flag
    Rover.picking_up = np.int(data["picking_up"])
    # Update number of rocks collected
    Rover.samples_collected = (
        Rover.samples_to_find - np.int(data["sample_count"])
    )

    print(
        'speed =', Rover.vel,
        'position =', Rover.pos,
        'throttle =', Rover.throttle,
        'steer_angle =', Rover.steer,
        'near_sample:', Rover.near_sample,
        'picking_up:', data["picking_up"],
        'sending pickup:', Rover.send_pickup,
        'total time:', Rover.total_time,
        'samples remaining:', data["sample_count"],
        'samples collected:', Rover.samples_collected
    )
    # Get the current image from the center camera of the rover
    imgString = data["image"]
    image = Image.open(BytesIO(base64.b64decode(imgString)))
    Rover.img = np.asarray(image)

    # Return updated Rover and separate image for optional saving
    return Rover, image


def create_output_images(Rover, Decider):
    """Create display output given worldmap results."""
    # Create a scaled map for plotting and clean up obs/nav pixels a bit
    if np.max(Rover.worldmap[:, :, 2]) > 0:
        nav_pix = Rover.worldmap[:, :, 2] > 0
        navigable = (Rover.worldmap[:, :, 2] *
                     (255 / np.mean(Rover.worldmap[nav_pix, 2])))
    else:
        navigable = Rover.worldmap[:, :, 2]

    if np.max(Rover.worldmap[:, :, 0]) > 0:
        obs_pix = Rover.worldmap[:, :, 0] > 0
        obstacle = (Rover.worldmap[:, :, 0] *
                    (255 / np.mean(Rover.worldmap[obs_pix, 0])))
    else:
        obstacle = Rover.worldmap[:, :, 0]

    likely_nav = navigable >= obstacle
    obstacle[likely_nav] = 0
    plotmap = np.zeros_like(Rover.worldmap)
    plotmap[:, :, 0] = obstacle
    plotmap[:, :, 2] = navigable
    plotmap = plotmap.clip(0, 255)
    # Overlay obstacle and navigable terrain map with ground truth map
    map_add = cv2.addWeighted(plotmap, 1, Rover.ground_truth, 0.5, 0)

    # Check whether any rock detections are present in worldmap
    rock_world_pos = Rover.worldmap[:, :, 1].nonzero()
    # If there are, we'll step through the known sample positions
    # to confirm whether detections are real
    samples_located = 0
    if rock_world_pos[0].any():
        rock_size = 2
        for idx in range(len(Rover.samples_pos[0])):
            test_rock_x = Rover.samples_pos[0][idx]
            test_rock_y = Rover.samples_pos[1][idx]
            rock_sample_dists = np.sqrt(
                (test_rock_x - rock_world_pos[1])**2 +
                (test_rock_y - rock_world_pos[0])**2
            )
            # If rocks were detected within 3 meters of known sample positions
            # consider it a success and plot the location of the known
            # sample on the map
            if np.min(rock_sample_dists) < 3:
                samples_located += 1
                map_add[test_rock_y-rock_size:test_rock_y+rock_size,
                        test_rock_x-rock_size:test_rock_x+rock_size, :] = 255

    # Calculate some statistics on the map results
    # First get the total number of pixels in the navigable terrain map
    tot_nav_pix = np.float(len((plotmap[:, :, 2].nonzero()[0])))
    # Next figure out how many of those correspond to ground truth pixels
    good_nav_pix = np.float(
        len(((plotmap[:, :, 2] > 0) &
            (Rover.ground_truth[:, :, 1] > 0)).nonzero()[0])
    )
    # Next find how many do not correspond to ground truth pixels
    bad_nav_pix = np.float(
        len(((plotmap[:, :, 2] > 0) &
            (Rover.ground_truth[:, :, 1] == 0)).nonzero()[0])
    )
    # Grab the total number of map pixels
    tot_map_pix = np.float(len((Rover.ground_truth[:, :, 1].nonzero()[0])))

    # Calculate % of ground truth map that has been successfully found
    Rover.perc_mapped = round(100*good_nav_pix / tot_map_pix, 1)

    # Calculate the number of good map pixel detections divided by total pixels
    # found to be navigable terrain
    if tot_nav_pix > 0:
        fidelity = round(100*good_nav_pix / (tot_nav_pix), 1)
    else:
        fidelity = 0

    # Flip the map for plotting so that the y-axis points upward in the display
    map_add = np.flipud(map_add).astype(np.float32)

    # NOTE: For more information, refer to OpenCV docs for putText function:
    #       http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # Add some text about map and rock sample detection results
    cv2.putText(map_add, "Time: ",
                (2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add, ""+str(np.round(Rover.total_time, 1))+' s',
                (39, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    cv2.putText(map_add, "Mapped: ",
                (2, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add, ""+str(Rover.perc_mapped)+'%',
                (57, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    cv2.putText(map_add, "Fidelity: ",
                (2, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add, ""+str(fidelity)+'%',
                (54, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    cv2.putText(map_add, "Rocks",
                (2, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    cv2.putText(map_add, " Located: ",
                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add, ""+str(samples_located),
                (74, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    cv2.putText(map_add, " Collected: ",
                (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add, ""+str(Rover.samples_collected),
                (80, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    # Add information about ReturnHome state on map display
    if Rover.going_home:
        cv2.putText(map_add, "Going Home:",
                    (2, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 255, 0), 1)

        cv2.putText(map_add, "distance: ",
                    (3, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        cv2.putText(map_add, ""+str(np.round(Rover.home_distance, 1))+' m',
                    (55, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)

        cv2.putText(map_add, "heading: ",
                    (3, 175),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
        cv2.putText(map_add, ""+str(np.round(Rover.home_heading, 1))+' deg',
                    (51, 175),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
    else:
        cv2.putText(map_add, "Mission:",
                    (2, 135),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 255, 255), 1)

        cv2.putText(map_add, "In Progress",
                    (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)

    # Add state information on rover vision display
    cv2.putText(Rover.vision_image, "> ",
                (5, 20), cv2.FONT_HERSHEY_COMPLEX, 0.53, (55, 255, 17), 2)

    cv2.putText(Rover.vision_image, "State:",
                (23, 20), cv2.FONT_HERSHEY_COMPLEX, 0.65, (55, 255, 17), 1)

    cv2.putText(Rover.vision_image, "_ "+Decider.curr_state.NAME.lower(),
                (97, 20), cv2.FONT_HERSHEY_COMPLEX, 0.65, (55, 255, 17), 1)

    cv2.putText(Rover.vision_image, "Robot Vision",
                (5, 151), cv2.FONT_HERSHEY_COMPLEX, 0.53, (255, 255, 255), 1)

    # Convert map and vision image to base64 strings for sending to server
    pil_img = Image.fromarray(map_add.astype(np.uint8))
    buff = BytesIO()
    pil_img.save(buff, format="JPEG")
    encoded_string1 = base64.b64encode(buff.getvalue()).decode("utf-8")

    pil_img = Image.fromarray(Rover.vision_image.astype(np.uint8))
    buff = BytesIO()
    pil_img.save(buff, format="JPEG")
    encoded_string2 = base64.b64encode(buff.getvalue()).decode("utf-8")

    return encoded_string1, encoded_string2
