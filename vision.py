import numpy as np
import cv2

def crop_map(img) :
    """crop the black borders out of the image to have a clean image to work on"""
    return True, cropped_img;

def extract_obstacles(img) :
    """Find all the black obstacles and extract their vertice from the image"""
    return True, obstacles;

def expand_obstacles(obstacles) :
    """expand the size of the ostacles to take the robot size in account for the path"""
    return obstacles;