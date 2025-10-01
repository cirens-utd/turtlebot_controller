import json
import numpy as np
import pandas as pd
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, LineString, Point, box
import os
from os import path, getcwd
from datetime import datetime
import zipfile
import tkinter as tk
from tkinter import filedialog
import argparse
import pdb
'''
This file will do an evaluation of runs completed. File stucture should be:
- ExperiementName (Consensus)
  - Trail1 (John)
    - YYYY-MM-DD-XXXXX0_robot1_Consensus.turtlebReplay
    - YYYY-MM-DD-XXXXX0_robot2_Consensus.turtlebReplay
    - ...
    - YYYY-MM-DD-XXXXX1_robot1_Consensus.turtlebReplay
    - YYYY-MM-DD-XXXXX1_robot2_Consensus.turtlebReplay
    - ...
  - Trail2 (Jane)
    ...

Assumptions: There will be folders for each trial. Inside each trial will have all files that need to be measured
    
Results will show:
Max, min, and adverage informaiton for each trail on 
- Total Displacement of Robot
- Total Distance Traveled
- Obstructions
- Time to complete
- Concensus Result
- Coverage Result
'''

def main():
    global turtle_replay_directory
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s", "--save", default=False, action="store_true", help="Save MP4")
    # parser.add_argument("-p", "--play", default=True, action="store_false", help="Set to not show graph")
    # parser.add_argument("-f", "--filename", default="Example", type=str, help="Name of MP4 file without .mp4")
    # parser.add_argument("-b", "--beauty", default=False, action="store_true", help="Save Pretty Json")
    # script_args = parser.parse_args()

    root = tk.Tk()
    root.withdraw()
    start_path = path.abspath(path.join(getcwd(), "..", "Replays"))
    if not path.exists(start_path):
        start_path = path.abspath(path.join(getcwd(), "Replays"))

    turtle_replay_directory = filedialog.askdirectory(
        title="Select Relay Directory",
        initialdir=start_path
    )

    root.destroy()

    result = {}
    if turtle_replay_directory:
        # Get name of all the trials we ran
        dirnames = [name for name in os.listdir(turtle_replay_directory) if os.path.isdir(os.path.join(turtle_replay_directory, name))]
        for name in dirnames:
            result[name] = {}

            path_name = os.path.join(turtle_replay_directory, name)
            for filename in os.listdir(path_name):
                if filename.split('.')[-1] == 'turtleReplay':
                    result[name][filename] = evaluate_replay(path_name, filename)


    # Adveraging the data
    final_result = {}
    for name, instance in result.items():
        final_result[name] = {}
        for run_name, info in instance.items():
            # Add robot to results
            if str(info['my_name']) not in final_result[name]:
                final_result[name][str(info['my_name'])] = {}
                final_result[name][str(info['my_name'])]['data'] = {
                    "controller": info['controller'],
                    "finished": [],
                    "displacement": [],
                    "total_distance": [],
                    "obstructions": [],
                    "my_completion_time": [],
                    "completion_time": [],
                    "consensus": {
                                    "experimental_error": [],
                                    "min_distance": [],
                                    "max_distance": [],
                                    "adv_distance": []
                                },
                    "coverage": {
                                    "computed": [], 
                                    "my_area": [],
                                    "total_area": [],
                                    "percentage_covered":[]
                                }
                }
                
            final_result[name][str(info['my_name'])]['data']['finished'].append(info['finished'])
            final_result[name][str(info['my_name'])]['data']['displacement'].append(info['displacement'])
            final_result[name][str(info['my_name'])]['data']['total_distance'].append(info['total_distance'])
            final_result[name][str(info['my_name'])]['data']['obstructions'].append(info['obstructions'])
            final_result[name][str(info['my_name'])]['data']['my_completion_time'].append(info['my_completion_time'])
            final_result[name][str(info['my_name'])]['data']['completion_time'].append(info['completion_time'])
            final_result[name][str(info['my_name'])]['data']['consensus']['experimental_error'].append(info['consensus']['experimental_error'])
            final_result[name][str(info['my_name'])]['data']['consensus']['min_distance'].append(info['consensus']['min_distance'])
            final_result[name][str(info['my_name'])]['data']['consensus']['max_distance'].append(info['consensus']['max_distance'])
            final_result[name][str(info['my_name'])]['data']['consensus']['adv_distance'].append(info['consensus']['adv_distance'])
            final_result[name][str(info['my_name'])]['data']['coverage']['computed'].append(info['coverage']['computed']) 
            if info['coverage']['computed']:
                final_result[name][str(info['my_name'])]['data']['coverage']['my_area'].append(info['coverage']['my_area'])
                final_result[name][str(info['my_name'])]['data']['coverage']['total_area'].append(info['coverage']['total_area'])
                final_result[name][str(info['my_name'])]['data']['coverage']['percentage_covered'].append(info['coverage']['percentage_covered'])
        final_result[name][str(info['my_name'])]['stats'] = {
            "controller": final_result[name][str(info['my_name'])]['data']['controller'],
            "times_run": len(final_result[name][str(info['my_name'])]['data']['finished']),
            "times_finished": np.array(final_result[name][str(info['my_name'])]['data']['finished']).sum(),
            "displacement": pd.Series(final_result[name][str(info['my_name'])]['data']['displacement']).describe(),
            "total_distance": pd.Series(final_result[name][str(info['my_name'])]['data']['total_distance']).describe(),
            "obstructions": pd.Series(final_result[name][str(info['my_name'])]['data']['obstructions']).describe(),
            "my_completion_time": pd.Series(final_result[name][str(info['my_name'])]['data']['my_completion_time']).describe(),
            "completion_time": pd.Series(final_result[name][str(info['my_name'])]['data']['completion_time']).describe(),
            "consensus": {
                "experimental_error": pd.Series(final_result[name][str(info['my_name'])]['data']['consensus']["experimental_error"]).describe(),
                "min_distance": pd.Series(final_result[name][str(info['my_name'])]['data']['consensus']["min_distance"]).describe(),
                "max_distance": pd.Series(final_result[name][str(info['my_name'])]['data']['consensus']["max_distance"]).describe(),
                "adv_distance": pd.Series(final_result[name][str(info['my_name'])]['data']['consensus']["adv_distance"]).describe(),
            },
            "coverage":{
                "times_computed": np.array(final_result[name][str(info['my_name'])]['data']['coverage']["computed"]).sum(),
                "my_area": pd.Series(final_result[name][str(info['my_name'])]['data']['coverage']["my_area"]).describe(),
                "total_area": pd.Series(final_result[name][str(info['my_name'])]['data']['coverage']["total_area"]).describe(),
                "percentage_covered": pd.Series(final_result[name][str(info['my_name'])]['data']['coverage']["percentage_covered"]).describe(),
            }
        }
        

    # Final Results
    pdb.set_trace()

def evaluate_replay(path_name, filename):
    data = extractData(path_name, filename)
    started = False
    finished = False

    # variables
    robot_name = None
    controller = None
    start_time = None
    start_position = None
    final_position = None
    neighbor_start_pose = None
    neighbor_final_pose = None
    concensus_point = None
    my_completion_time = None
    completion_time = None
    obstructed = False

    total_distance = 0
    obstructions = 0

    for line in data:
        for entry in line:
            if not started:
                # robot is now starting. Get starting information
                if entry['robot_moving']:
                    started = True
                    start_time = entry['time']
                    # robot_name = entry['my_name']
                    # controller = entry['mainClass']
                    start_position = np.array([entry['my_pose']['pose']['position']['x'], entry['my_pose']['pose']['position']['y']])
                    prev_pose = start_position

                    # Saving Neighbors Info
                    neighbor_start_pose = {}
                    for name, pose in entry['neighbor_poses'].items():
                        neighbor_start_pose[name] = np.array([
                            pose['pose']['position']['x'],
                            pose['pose']['position']['y']
                        ])
            elif not finished:
                # time this robot finished moving but still waiting on neighbors
                if entry['motion_complete'] and my_completion_time == None:
                    my_completion_time = entry['time']
                else:
                    my_completion_time = None
                
                # Tracking Distance Traveled
                current_pose = np.array([entry['my_pose']['pose']['position']['x'], entry['my_pose']['pose']['position']['y']]) 
                total_distance += np.round(np.linalg.norm(current_pose - prev_pose), 2)

                 # time total process is done
                if entry['neighbors_complete']:
                    finshed = True

                    completion_time = entry['time']
                    final_position = current_pose

                    # Saving Neighbors Info
                    neighbor_final_pose = {}
                    for name, pose in entry['neighbor_poses'].items():
                        neighbor_final_pose[name] = np.array([
                            pose['pose']['position']['x'],
                            pose['pose']['position']['y']
                        ])

                # Tracking Obstructions
                if not obstructed and entry['path_obstructed']:
                    obstructed = True
                    obstructions += 1
                elif obstructed and not entry['path_obstructed']:
                    obstructed = False

                prev_pose = current_pose

        
    # Controller did not finish. Get last known position
    if type(my_completion_time) == type(None):
        my_completion_time = entry['time']

    if type(final_position) == type(None):
        completion_time = entry['time']
        final_position = current_pose
        
        # Saving Neighbors Info
        neighbor_final_pose = {}
        for name, pose in entry['neighbor_poses'].items():
            neighbor_final_pose[name] = np.array([
                pose['pose']['position']['x'],
                pose['pose']['position']['y']
            ])

    start_time = datetime.strptime(start_time, "%Y-%m-%d.%H%M%S")
    my_completion_time = datetime.strptime(my_completion_time, "%Y-%m-%d.%H%M%S")
    completion_time = datetime.strptime(completion_time, "%Y-%m-%d.%H%M%S")

    # pool in results
    return {
        "my_name": robot_name,
        "controller": controller,
        "finished": finished,
        "displacement": np.round(np.linalg.norm(final_position - start_position), 2),
        "total_distance": total_distance,
        "obstructions": obstructions,
        "my_completion_time": my_completion_time - start_time,
        "completion_time": completion_time - start_time,
        "consensus": find_consensus_result(start_position, neighbor_start_pose, final_position, neighbor_final_pose),
        "coverage": find_coverage_result(final_position, neighbor_final_pose)
    }


def extractData(path_name, filename):

    full_filename = os.path.join(path_name, filename)
    with zipfile.ZipFile(full_filename, 'r') as zip_ref:
        zip_ref.extractall("usable_replay")

    file_name = os.listdir(r"usable_replay/")[0]

    data = []

    with open(r"usable_replay/" + file_name, 'r', errors="ignore") as curFile:
        file_content = curFile.read()
        json_arrays = file_content.strip().split("\n")
        for json_array in json_arrays:
            data.append(json.loads(json_array))

    os.remove(r"usable_replay/" + file_name)
    os.rmdir(r"usable_replay/")

    return data

def find_consensus_result(start_position, neighbor_start_position, final_position, neighbor_final_position):

    # finding theoretical convergence point
    consensus_point = start_position
    total = 1
    for name, pose in neighbor_start_position.items():
        consensus_point += pose
        total += 1
    consensus_point /= total

    # finding how far away from my neighbors I am
    distance = 0
    max_distance = 0
    min_distance = 10000
    for name, pose in neighbor_final_position.items():
        difference = np.round(np.linalg.norm(final_position - pose), 2)
        distance += difference
        if max_distance < difference:
            max_distance = difference
        if min_distance > difference:
            min_distance = difference
    distance /= total 

    return {
        "experimental_error": np.round(np.linalg.norm(final_position - consensus_point), 2),
        "min_distance": min_distance,
        "max_distance": max_distance,
        "adv_distance": distance
    }

def bounded_voronoi_region(vor, point_idx, bounding_shape, far_enough=100):
    region_idx = vor.point_region[point_idx]
    region = vor.regions[region_idx]
    if -1 in region or len(region) == 0:
        # Region is infinite, reconstruct polygon with ridge directions
        point = vor.points[point_idx]
        ridges = [r for r in vor.ridge_points if point_idx in r]
        lines = []
        for (p1_idx, p2_idx), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
            if point_idx not in (p1_idx, p2_idx):
                continue
            if v1 == -1 or v2 == -1:
                # Get the two original points that generated this ridge
                p1 = vor.points[p1_idx]
                p2 = vor.points[p2_idx]

                # Compute midpoint of p1 and p2 (for visualizing or anchoring)
                midpoint = (p1 + p2) / 2

                # Compute the vector between them
                line_vec = p2 - p1

                # Compute the normal (perpendicular vector)
                direction = np.array([-line_vec[1], line_vec[0]])
                normal = direction / np.linalg.norm(direction)  # Normalize

                # Choose Direction
                finite_vertex = vor.vertices[v1 if v1 != -1 else v2]
                to_vertex = finite_vertex - midpoint

                # If dot product is negative, we are going the wrong way
                if np.dot(to_vertex, normal) > 0:
                    normal *= -1 
                
                far_point = finite_vertex + far_enough * normal
                line = LineString([finite_vertex, far_point])

            else:
                line = LineString([vor.vertices[v1], vor.vertices[v2]])
            lines.append(line)
        try:
            region_poly = Polygon(LineString(np.concatenate([l.coords for l in lines])).convex_hull)
        except Exception as e:
            print(e)
            pdb.set_trace()
            return None
    else:
        region_poly = Polygon([vor.vertices[i] for i in region])
    
    # Clip with boundry
    return region_poly.intersection(bounding_shape)

def find_coverage_result(my_position, neighbor_position):
    # find the percentage of the area we are covering
    boundary_points = [(-3.8, 0.19), (-0.12, 3.36), (2.69, 2.0), (2.74, -2.6), (-0.18, -5.0), (-2.36, -5.12)]

    all_points = [my_position]
    for name, pose in neighbor_position.items():
        all_points.append(pose)

    if len(all_points) < 4:
        return {
            "computed": False
        }

    vor = Voronoi(all_points)
    boundary = Polygon(boundary_points)

    my_area = 0
    total_area = boundary.area

    region_poly = bounded_voronoi_region(vor, 0, boundary)
    if type(region_poly) != type(None):
        my_area = region_poly.area

    return {
        "computed": True, 
        "my_area": my_area,
        "total_area": total_area,
        "percentage_covered": np.round(my_area/total_area, 3) * 100
    }

if __name__ == '__main__':
    main()