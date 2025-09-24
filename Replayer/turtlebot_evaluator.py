import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, Slider
import matplotlib.animation as animation
import numpy as np
import os
from os import path, getcwd
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

            for filename in os.listdir(os.path.join(turtle_replay_directory, name)):
                if filename.split('.')[-1] == 'turtleReplay':
                    evaluate_replay(filename)

def evaluate_replay(filename):
    data = extractData(filename)
    started = False
    finished = False

    # variables
    start_time = None
    start_position = None
    final_position = None
    neighbor_start_pose = None
    neighbor_final_pose = None
    concensus_point = None
    my_completion_time = None
    completion_time = None
    obsturcted = False

    total_distance = 0
    obstructions = 0

    for line in data:
        for entry in line:
            if not started:
                # robot is now starting. Get starting information
                if entry['robot_moving']:
                    started = True
                    start_time = entry['time']
                    start_position = np.array([entry['my_pose']['pose']['position']['x'], entry['my_pose']['pose']['position']['y']])
                    prev_pose = start_position

                    # Saving Neighbors Info
                    neighbor_start_pose = {}
                    concensus_x = entry['my_pose']['pose']['position']['x']
                    concensus_y = entry['my_pose']['pose']['position']['y']
                    total = 1
                    for name, pose in entry['neighbor_poses'].items():
                        neighbor_start_pose[name] = np.array([
                            'x': pose['pose']['position']['x'],
                            'y': pose['pose']['position']['y']
                        ])
                        concensus_x += pose['pose']['position']['x']
                        concensus_y += pose['pose']['position']['y']
                        total += 1
                    consensus_point = np.array([concensus_x /= total, concensus_y /= total])
            elif not finished:
                # time this robot finished moving but still waiting on neighbors
                if motion_complete and my_completion_time == None:
                    my_completion_time = entry['time']
                else:
                    my_completion_time = None

                # time total process is done
                if neighbors_complete:
                    finshed = True

                    completion_time = entry['time']
                    final_position = np.array([entry['my_pose']['pose']['position']['x'], entry['my_pose']['pose']['position']['y']])

                    # Saving Neighbors Info
                    neighbor_final_pose = {}
                    for name, pose in entry['neighbor_poses'].items():
                        neighbor_final_pose[name] = np.array([
                            'x': pose['pose']['position']['x'],
                            'y': pose['pose']['position']['y']
                        ])
                
                # Tracking Distance Traveled
                current_pos = np.array([entry['my_pose']['pose']['position']['x'], entry['my_pose']['pose']['position']['y']])        
                total_distance += np.round(np.linalg.norm(current_pos, prev_pose), 2)

                # Tracking Obstructions
                if not obstructed and entry['path_obstructed']:
                    obstructed = True
                    obstructions += 1
                elif obstructed and not entry['path_obstructed']:
                    obstructed = False

                prev_pose = current_pose
            
    # pool in results
    return{
        "displacemnt": np.linalg.norm(final_position, start_position),
        "total_distance": total_distance,
        "obstructions": obstructions,
        "my_completion_time": my_completion_time - start_time,
        "completion_time": completion_time - start_time,
        "consensus_delta": np.linalg.norm(final_position, consensus_point),
        "coverage_percent": find_coverage_delta()
    }


def extractData(filename):
    with zipfile.ZipFile(filename, 'r') as zip_ref:
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

def find_coverage_delta():
    return 100