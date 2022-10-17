#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
import os, yaml, math
import matplotlib.pyplot as plt


def dist(pos,e):
    math.sqrt((pos[e[0]][0]-pos[e[1]][0])**2+(pos[e[0]][1]-pos[e[1]][1])**2)

multi_turtlesim_traffic_path = get_package_share_directory()
traffic_path = os.path.join(multi_turtlesim_traffic_path,)

with open(traffic_path) as f:
    traffic_info = yaml.load(f, Loader=yaml.loader.SafeLoader)