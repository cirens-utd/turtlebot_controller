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
now = datetime.now().strftime("%Y-%m-%d-%H%M%S")
robot_file_summary = f"{now}_Each_Robot_Summary.json"
general_file_summary = f"{now}_Robot_Summary.json"

def main():
    global turtle_replay_directory

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--save", default=False, action="store_true", help="Save Files")
    script_args = parser.parse_args()

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

    result = modify_names(result)

    # Adveraging the data
    final_result, all_result = get_statistics(result)
    
    # Final Results
    final_file_data, final_file_summary = create_files(final_result, all_result, script_args.save)

    display_results(final_file_data, final_file_summary)

    print("\n\nPress c to close program.")
    pdb.set_trace()

def display_results(final_data, final_summary):

    for name, value in final_summary.items():
        print(
    f"""
    General Robot Stats For {name}:
        Controller: {final_summary[name]['Autonomous Robot Stats']['Controller']}
        Times Run: {final_summary[name]['All Robot Stats']['Times Run']}
        Times Finished: {final_summary[name]['All Robot Stats']['Times Finished']}
        Displacement: {final_summary[name]['All Robot Stats']['Displacement']['Mean']}
        Total Distance: {final_summary[name]['All Robot Stats']['Total Distance']['Mean']}
        Obstructions: {final_summary[name]['All Robot Stats']['Obstructions']['Mean']}
        Completion Time: {final_summary[name]['All Robot Stats']['Completion Time']['Mean']}
        Consensus: 
            Experimental Error: {final_summary[name]['All Robot Stats']['Consensus']['Experimental Error']['Mean']}
            Adv Distance: {final_summary[name]['All Robot Stats']['Consensus']['Adv Distance']['Mean']}
        Coverage: 
            Times Computed: {final_summary[name]['All Robot Stats']['Coverage']['Times Computed']}
            My Area: {final_summary[name]['All Robot Stats']['Coverage']['My Area']['Mean']}
            Total Area: {final_summary[name]['All Robot Stats']['Coverage']['Total Area']['Mean']}
            Percentage Covered: {final_summary[name]['All Robot Stats']['Coverage']['Percentage Covered']['Mean']}
    """)

def create_files(final_result, all_result, write=False):
    global robot_file_summary
    global general_file_summary

    # make a dictionary of the data we want and jsonify it
    final_file_data = {}
    final_file_summary = {}
    for name, robot_info in final_result.items():
        final_file_data[name] = {}
        final_file_summary[name] = {}
        for robot, run_info in robot_info.items():
            final_file_data[name][robot] = {}

            # individual Robots
            for key, value in run_info['stats'].items():
                final_file_data[name][robot][key.replace('_', ' ').title()] = {}
                if key in ['controller', 'times_run', 'times_finished']:
                    if type(value) == type(np.int64(1)):
                        final_file_data[name][robot][key.replace('_', ' ').title()] = value.item()
                    else:
                        final_file_data[name][robot][key.replace('_', ' ').title()] = value
                elif key in ['consensus', 'coverage']:
                    for key2, value2 in value.items():
                        final_file_data[name][robot][key.replace('_', ' ').title()][key2.replace('_', ' ').title()] = {}
                        if key2 in ['times_computed']:
                            if type(value2) == type(np.int64(1)):
                                final_file_data[name][robot][key.replace('_', ' ').title()][key2.replace('_', ' ').title()] = value2.item()
                            else:
                                final_file_data[name][robot][key.replace('_', ' ').title()][key2.replace('_', ' ').title()] = value2
                        else:
                            for key3, value3 in value2.items():
                                if key3 in ['mean', 'std', 'min', 'max']:
                                    if type(value3) == type(pd.Timedelta(0)):
                                        final_file_data[name][robot][key.replace('_', ' ').title()][key2.replace('_', ' ').title()][key3.capitalize()] = value3.total_seconds()
                                    elif type(value2) == type(np.int64(1)):
                                        final_file_data[name][robot][key.replace('_', ' ').title()][key2.replace('_', ' ').title()][key3.capitalize()] = value3.item()
                                    else:
                                        final_file_data[name][robot][key.replace('_', ' ').title()][key2.replace('_', ' ').title()][key3.capitalize()] = value3
                else:
                    for key2, value2 in value.items():
                        if key2 in ['mean', 'std', 'min', 'max']:
                            if type(value2) == type(pd.Timedelta(0)):
                                final_file_data[name][robot][key.replace('_', ' ').title()][key2.capitalize()] = value2.total_seconds()
                            elif type(value2) == type(np.int64(1)):
                                final_file_data[name][robot][key.replace('_', ' ').title()][key2.capitalize()] = value2.item()
                            else:
                                final_file_data[name][robot][key.replace('_', ' ').title()][key2.capitalize()] = value2

        # All robot information
        for stats in ['stats', 'auto_stats', 'manual_stats']:
            if stats == 'stats':
                stats_title = "All Robot Stats"
            elif stats == 'auto_stats':
                stats_title = "Autonomous Robot Stats"
            elif stats == 'manual_stats':
                stats_title = "Manual Robot Stats"
            final_file_summary[name][stats_title] = {}

            for key, value in all_result[name][stats].items():
                final_file_summary[name][stats_title][key.replace('_', ' ').title()] = {}
                if key in ['controller', 'times_run', 'times_finished']:
                    if type(value) == type(np.int64(1)):
                        final_file_summary[name][stats_title][key.replace('_', ' ').title()] = value.item()
                    else:
                        final_file_summary[name][stats_title][key.replace('_', ' ').title()] = value
                elif key in ['consensus', 'coverage']:
                    for key2, value2 in value.items():
                        final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.replace('_', ' ').title()] = {}
                        if key2 in ['times_computed']:
                            if type(value2) == type(np.int64(1)):
                                final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.replace('_', ' ').title()] = value2.item()
                            else:
                                final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.replace('_', ' ').title()] = value2
                        else:
                            for key3, value3 in value2.items():
                                if key3 in ['mean', 'std', 'min', 'max']:
                                    if type(value3) == type(pd.Timedelta(0)):
                                        final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.replace('_', ' ').title()][key3.capitalize()] = value3.total_seconds()
                                    elif type(value2) == type(np.int64(1)):
                                        final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.replace('_', ' ').title()][key3.capitalize()] = value3.item()
                                    else:
                                        final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.replace('_', ' ').title()][key3.capitalize()] = value3
                else:
                    for key2, value2 in value.items():
                        if key2 in ['mean', 'std', 'min', 'max']:
                            if type(value2) == type(pd.Timedelta(0)):
                                final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.capitalize()] = value2.total_seconds()
                            elif type(value2) == type(np.int64(1)):
                                final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.capitalize()] = value2.item()
                            else:
                                final_file_summary[name][stats_title][key.replace('_', ' ').title()][key2.capitalize()] = value2
        

    if write:
        with open(robot_file_summary, 'w') as file:
            file.write(json.dumps(final_file_data, indent=2))
        with open(general_file_summary, 'w') as file:
            file.write(json.dumps(final_file_summary, indent=2))

    return final_file_data, final_file_summary

def modify_names(result):
    '''
    result: <class 'dict'> {
        "folder_name": <class 'dict'> {
            "file_name": <class 'dict'> {
                "my_name": <class 'str'> -> name of robot
                "controller": <class 'str'> -> name of robot controller
                "finished": <class 'bool'> -> boolean if robot finished
                "displacement": <class 'numpy.float64'> -> total distance between the start and end position
                "total_distance": <class 'numpy.float64'> -> total distance robot traveled
                "obstructions": <class 'int'> -> number of times the robot was obstructed
                "my_completion_time": <class 'datetime.timedelta'> -> Time this robot finished
                "completion_time": <class 'datetime.timedelta'> -> Time all the robots finished
                "consensus": <class 'dict'> {
                    "experimental_error": <class 'numpy.float64'> -> Error between calculated consesus point and actual ending point
                    "min_distance": <class 'numpy.float64'> -> min distance to a neighbor
                    "max_distance": <class 'numpy.float64'> -> max distance to a neighbor
                    "adv_distance": <class 'numpy.float64'> -> adverage distance to a neighbor
                }
                "coverage": <class 'dict'> {
                    "computed": <class 'bool'> -> determine if coverage was computed. If not, the other keys will not exist 
                    "my_area": <class 'float'> -> area this robot covers
                    "total_area": <class 'float'> -> total area in region
                    "percentage_covered": <class 'numpy.float64'> -> percentage of the area this robot covers
                }
            }
        }
    }

    return: Same as return but change robot name to prefered name
    '''
    # lining up robot names
    for name, instance in result.items():
        names = {}
        change_names = {}
        file_overwrite = False

        for run_name, info in instance.items():
            # tracking names used
            if str(info['my_name']) not in names:
                names[str(info['my_name'])] = 0
            names[str(info['my_name'])] += 1

        count = 0
        miss_match = False
        prev_name = None
        miss_match_names = []
        print(f"\nFor {name} the count for each robot:")
        for robot_name, num in names.items():
            print(f"{robot_name}: {num}")
            if not count:
                count = num
                first_name = robot_name
            elif count != num:
                if count < num:
                    count = num
                    miss_match_names.append(prev_name)
                    prev_name = robot_name
                else:
                    miss_match_names.append(robot_name)
                miss_match = True

        print("")

        if miss_match:
            response = input(f"\nFor {name}, do you want to match up these name: {', '.join(miss_match_names)}? (Y or N)\n")
            while response.upper() == "Y":
                change_names = {}
                for old_name in miss_match_names:
                    new_name = input(f"Please put new name for {old_name}: ")
                    if new_name:
                        change_names[old_name] = new_name

                print(f"\nThe names we could change were: {', '.join(miss_match_names)}")
                print("The names that will be changed are:")
                for old_name, new_name in change_names.items():
                    print(f"{old_name} -> {new_name}")

                response = input(f"Would you like to attempt to change the names again?\nY - Repeat or N - End\n")

            if change_names:
                response = input("\nWould you like to save this change into the file? (Y or N)\n")
                if response.upper() == "Y":
                    file_overwrite = True
                instance = modify_data(name, instance, change_names, file_overwrite)


    return result

def modify_data(folder, data, names, overwrite=False):
    '''
    folder: <class 'str'> -> name of the folder the file belongs too
    data: <class 'dict'> {
        <class 'str'> "filename": {
            "my_name": <class 'str'> -> name of robot
            "controller": <class 'str'> -> name of robot controller
            "finished": <class 'bool'> -> boolean if robot finished
            "displacement": <class 'numpy.float64'> -> total distance between the start and end position
            "total_distance": <class 'numpy.float64'> -> total distance robot traveled
            "obstructions": <class 'int'> -> number of times the robot was obstructed
            "my_completion_time": <class 'datetime.timedelta'> -> Time this robot finished
            "completion_time": <class 'datetime.timedelta'> -> Time all the robots finished
            "consensus": <class 'dict'> {
                "experimental_error": <class 'numpy.float64'> -> Error between calculated consesus point and actual ending point
                "min_distance": <class 'numpy.float64'> -> min distance to a neighbor
                "max_distance": <class 'numpy.float64'> -> max distance to a neighbor
                "adv_distance": <class 'numpy.float64'> -> adverage distance to a neighbor
            }
            "coverage": <class 'dict'> {
                "computed": <class 'bool'> -> determine if coverage was computed. If not, the other keys will not exist 
                "my_area": <class 'float'> -> area this robot covers
                "total_area": <class 'float'> -> total area in region
                "percentage_covered": <class 'numpy.float64'> -> percentage of the area this robot covers
            }
        },
        ...
    }
    names: <class 'dict'> {
        <class 'str'> old_name1: <class 'str'> new_name1,
        <class 'str'> old_name2: <class 'str'> new_name2,
        ...
    }
    overwrite: <class 'bool'> -> Boolean to overwrite the original file

    return data -> same as passed in but add key original_name as the original my_name and replace my_name with new value
    '''

    for run_name, info in data.items():
        if str(info['my_name']) in names.keys():
            info['original_name'] = info['my_name']
            info['my_name'] = names[str(info['my_name'])]
            if overwrite:
                save_new_file(folder, run_name, info['my_name'])

    return data

def save_new_file(folder, zip_file_name, new_robot_name):
    '''
    folder: <class 'str'> name of folder
    zipfile_name: <class 'str'> name of zip file
    new_robot_name: <class 'str'> new name for robot
    '''
    global turtle_replay_directory

    print(f"Change File {zip_file_name} to use {new_robot_name}")

    full_filename = os.path.join(turtle_replay_directory, folder, zip_file_name)
    with zipfile.ZipFile(full_filename, 'r') as zip_ref:
        zip_ref.extractall("usable_replay")

    file_name = os.listdir(r"usable_replay/")[0]

    data = []
    with open(r"usable_replay/" + file_name, 'r', errors="ignore") as curFile:
        file_content = curFile.read()
        json_arrays = file_content.strip().split("\n")
        for json_array in json_arrays:
            data.append(json.loads(json_array))

    for line in data:
        for entry in line:
            entry['original_name'] = entry['my_name']
            entry['my_name'] = new_robot_name

    with open(r"usable_replay/" + file_name, 'w') as file:
        file.write(json.dumps(data.pop(0)) + "\n")
    
    with open(r"usable_replay/" + file_name, 'a+') as file:
        for line in data:
            file.write(json.dumps(line) + "\n")
    

    jungle_zip = zipfile.ZipFile(full_filename, 'w')
    jungle_zip.write(r"usable_replay/" + file_name, arcname=file_name, compress_type=zipfile.ZIP_DEFLATED)
    print(f"\n***********\nFile saved: {zip_file_name}\n***********\n")

    os.remove(r"usable_replay/" + file_name)
    os.rmdir(r"usable_replay/")

def get_statistics(result):
    '''
    results: {
        "folder": {
            "file_name": {
                *** Results from this file. Both by robot and then all together
                See evaluate_replay() for details
            },
            ...
        }
    }

    ** Return two dictionaries. On for the overall statistics and one for each robot
    return <class 'dict'> {
        "foldername":{
            "data":{
                "controller": <class 'str'> -> name of controller
                "finished": <class 'list'> -> list of booleans showing if the robot finished that run
                "displacement": <class 'list'> -> list of displacement results from each run
                "total_distance": <class 'list'> -> list of total distance results from each run
                "obstructions": <class 'list'> -> list of obstructions from each run
                "my_completion_time": <class 'list'> -> list of completion time from each run
                "completion_time": <class 'list'> -> list of completion_time from each run
                "consensus": {
                                "experimental_error": <class 'list'> -> list of experimental errors from each run
                                "min_distance": <class 'list'> -> list of minimum distance to their partner from each run
                                "max_distance": <class 'list'> -> list of maximum distance to their partner from each run
                                "adv_distance": <class 'list'> -> list of adverage distance to their partner from each run
                            },
                "coverage": {
                                "computed": <class 'list'> -> list of booleans from each run, weather or not coverage was computed
                                "my_area": <class 'list'> -> list of area this robot covered for each run
                                "total_area": <class 'list'> -> list of total area from each run
                                "percentage_covered": <class 'list'> -> list of percentage of the total area from each run
                            }
            },
            "stats":{
                *** Ran discribe method in a pd.Series for each of the list above.
                <class 'pandas.core.series.Series'>
                count: number of runs
                mean: mean of values
                std: std of values
                min: min of values
                25%: 25th percental
                50%: 50th percental
                75%: 75th percental
                max: max of values
            }
        }
    }
    '''

    # get statistic data from replays
    final_result = {}
    all_result = {}
    for name, instance in result.items():
        final_result[name] = {}
        all_result[name] = {}
        all_result[name]['data'] = {"controller": [],"finished": [],"displacement": [],"total_distance": [],"obstructions": [],"my_completion_time": [],"completion_time": [],"consensus": {"experimental_error": [],"min_distance": [],"max_distance": [],"adv_distance": []},"coverage": {"computed": [], "my_area": [],"total_area": [],"percentage_covered":[]}
        }
        all_result[name]['auto_data'] = {"controller": '',"finished": [],"displacement": [],"total_distance": [],"obstructions": [],"my_completion_time": [],"completion_time": [],"consensus": {"experimental_error": [],"min_distance": [],"max_distance": [],"adv_distance": []},"coverage": {"computed": [], "my_area": [],"total_area": [],"percentage_covered":[]}
        }
        all_result[name]['manual_data'] = {"controller": '',"finished": [],"displacement": [],"total_distance": [],"obstructions": [],"my_completion_time": [],"completion_time": [],"consensus": {"experimental_error": [],"min_distance": [],"max_distance": [],"adv_distance": []},"coverage": {"computed": [], "my_area": [],"total_area": [],"percentage_covered":[]}
        }
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
                
            # Individual data
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

            # Overall Data
            if info['controller'] not in all_result[name]['data']['controller']:
                all_result[name]['data']['controller'].append(info['controller'])
            all_result[name]['data']['finished'].append(info['finished'])
            all_result[name]['data']['displacement'].append(info['displacement'])
            all_result[name]['data']['total_distance'].append(info['total_distance'])
            all_result[name]['data']['obstructions'].append(info['obstructions'])
            all_result[name]['data']['my_completion_time'].append(info['my_completion_time'])
            all_result[name]['data']['completion_time'].append(info['completion_time'])
            all_result[name]['data']['consensus']['experimental_error'].append(info['consensus']['experimental_error'])
            all_result[name]['data']['consensus']['min_distance'].append(info['consensus']['min_distance'])
            all_result[name]['data']['consensus']['max_distance'].append(info['consensus']['max_distance'])
            all_result[name]['data']['consensus']['adv_distance'].append(info['consensus']['adv_distance'])
            all_result[name]['data']['coverage']['computed'].append(info['coverage']['computed']) 
            if info['coverage']['computed']:
                all_result[name]['data']['coverage']['my_area'].append(info['coverage']['my_area'])
                all_result[name]['data']['coverage']['total_area'].append(info['coverage']['total_area'])
                all_result[name]['data']['coverage']['percentage_covered'].append(info['coverage']['percentage_covered'])

            # Autonomous Data
            if info['controller'] != 'ViewerAgent':
                all_result[name]['auto_data']['controller'] = info['controller']
                all_result[name]['auto_data']['finished'].append(info['finished'])
                all_result[name]['auto_data']['displacement'].append(info['displacement'])
                all_result[name]['auto_data']['total_distance'].append(info['total_distance'])
                all_result[name]['auto_data']['obstructions'].append(info['obstructions'])
                all_result[name]['auto_data']['my_completion_time'].append(info['my_completion_time'])
                all_result[name]['auto_data']['completion_time'].append(info['completion_time'])
                all_result[name]['auto_data']['consensus']['experimental_error'].append(info['consensus']['experimental_error'])
                all_result[name]['auto_data']['consensus']['min_distance'].append(info['consensus']['min_distance'])
                all_result[name]['auto_data']['consensus']['max_distance'].append(info['consensus']['max_distance'])
                all_result[name]['auto_data']['consensus']['adv_distance'].append(info['consensus']['adv_distance'])
                all_result[name]['auto_data']['coverage']['computed'].append(info['coverage']['computed']) 
                if info['coverage']['computed']:
                    all_result[name]['auto_data']['coverage']['my_area'].append(info['coverage']['my_area'])
                    all_result[name]['auto_data']['coverage']['total_area'].append(info['coverage']['total_area'])
                    all_result[name]['auto_data']['coverage']['percentage_covered'].append(info['coverage']['percentage_covered'])
            else:
                # Human Data
                all_result[name]['manual_data']['controller'] = info['controller']
                all_result[name]['manual_data']['finished'].append(info['finished'])
                all_result[name]['manual_data']['displacement'].append(info['displacement'])
                all_result[name]['manual_data']['total_distance'].append(info['total_distance'])
                all_result[name]['manual_data']['obstructions'].append(info['obstructions'])
                all_result[name]['manual_data']['my_completion_time'].append(info['my_completion_time'])
                all_result[name]['manual_data']['completion_time'].append(info['completion_time'])
                all_result[name]['manual_data']['consensus']['experimental_error'].append(info['consensus']['experimental_error'])
                all_result[name]['manual_data']['consensus']['min_distance'].append(info['consensus']['min_distance'])
                all_result[name]['manual_data']['consensus']['max_distance'].append(info['consensus']['max_distance'])
                all_result[name]['manual_data']['consensus']['adv_distance'].append(info['consensus']['adv_distance'])
                all_result[name]['manual_data']['coverage']['computed'].append(info['coverage']['computed']) 
                if info['coverage']['computed']:
                    all_result[name]['manual_data']['coverage']['my_area'].append(info['coverage']['my_area'])
                    all_result[name]['manual_data']['coverage']['total_area'].append(info['coverage']['total_area'])
                    all_result[name]['manual_data']['coverage']['percentage_covered'].append(info['coverage']['percentage_covered'])

        for run_name, info in instance.items():
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
            all_result[name]['stats'] = {
                "controller": all_result[name]['data']['controller'],
                "times_run": len(all_result[name]['data']['finished']),
                "times_finished": np.array(all_result[name]['data']['finished']).sum(),
                "displacement": pd.Series(all_result[name]['data']['displacement']).describe(),
                "total_distance": pd.Series(all_result[name]['data']['total_distance']).describe(),
                "obstructions": pd.Series(all_result[name]['data']['obstructions']).describe(),
                "my_completion_time": pd.Series(all_result[name]['data']['my_completion_time']).describe(),
                "completion_time": pd.Series(all_result[name]['data']['completion_time']).describe(),
                "consensus": {
                    "experimental_error": pd.Series(all_result[name]['data']['consensus']["experimental_error"]).describe(),
                    "min_distance": pd.Series(all_result[name]['data']['consensus']["min_distance"]).describe(),
                    "max_distance": pd.Series(all_result[name]['data']['consensus']["max_distance"]).describe(),
                    "adv_distance": pd.Series(all_result[name]['data']['consensus']["adv_distance"]).describe(),
                },
                "coverage":{
                    "times_computed": np.array(all_result[name]['data']['coverage']["computed"]).sum(),
                    "my_area": pd.Series(all_result[name]['data']['coverage']["my_area"]).describe(),
                    "total_area": pd.Series(all_result[name]['data']['coverage']["total_area"]).describe(),
                    "percentage_covered": pd.Series(all_result[name]['data']['coverage']["percentage_covered"]).describe(),
                }
            }
            all_result[name]['auto_stats'] = {
                "controller": all_result[name]['auto_data']['controller'],
                "times_run": len(all_result[name]['auto_data']['finished']),
                "times_finished": np.array(all_result[name]['auto_data']['finished']).sum(),
                "displacement": pd.Series(all_result[name]['auto_data']['displacement']).describe(),
                "total_distance": pd.Series(all_result[name]['auto_data']['total_distance']).describe(),
                "obstructions": pd.Series(all_result[name]['auto_data']['obstructions']).describe(),
                "my_completion_time": pd.Series(all_result[name]['auto_data']['my_completion_time']).describe(),
                "completion_time": pd.Series(all_result[name]['auto_data']['completion_time']).describe(),
                "consensus": {
                    "experimental_error": pd.Series(all_result[name]['auto_data']['consensus']["experimental_error"]).describe(),
                    "min_distance": pd.Series(all_result[name]['auto_data']['consensus']["min_distance"]).describe(),
                    "max_distance": pd.Series(all_result[name]['auto_data']['consensus']["max_distance"]).describe(),
                    "adv_distance": pd.Series(all_result[name]['auto_data']['consensus']["adv_distance"]).describe(),
                },
                "coverage":{
                    "times_computed": np.array(all_result[name]['auto_data']['coverage']["computed"]).sum(),
                    "my_area": pd.Series(all_result[name]['auto_data']['coverage']["my_area"]).describe(),
                    "total_area": pd.Series(all_result[name]['auto_data']['coverage']["total_area"]).describe(),
                    "percentage_covered": pd.Series(all_result[name]['auto_data']['coverage']["percentage_covered"]).describe(),
                }
            }
            all_result[name]['manual_stats'] = {
                "controller": all_result[name]['manual_data']['controller'],
                "times_run": len(all_result[name]['manual_data']['finished']),
                "times_finished": np.array(all_result[name]['manual_data']['finished']).sum(),
                "displacement": pd.Series(all_result[name]['manual_data']['displacement']).describe(),
                "total_distance": pd.Series(all_result[name]['manual_data']['total_distance']).describe(),
                "obstructions": pd.Series(all_result[name]['manual_data']['obstructions']).describe(),
                "my_completion_time": pd.Series(all_result[name]['manual_data']['my_completion_time']).describe(),
                "completion_time": pd.Series(all_result[name]['manual_data']['completion_time']).describe(),
                "consensus": {
                    "experimental_error": pd.Series(all_result[name]['manual_data']['consensus']["experimental_error"]).describe(),
                    "min_distance": pd.Series(all_result[name]['manual_data']['consensus']["min_distance"]).describe(),
                    "max_distance": pd.Series(all_result[name]['manual_data']['consensus']["max_distance"]).describe(),
                    "adv_distance": pd.Series(all_result[name]['manual_data']['consensus']["adv_distance"]).describe(),
                },
                "coverage":{
                    "times_computed": np.array(all_result[name]['manual_data']['coverage']["computed"]).sum(),
                    "my_area": pd.Series(all_result[name]['manual_data']['coverage']["my_area"]).describe(),
                    "total_area": pd.Series(all_result[name]['manual_data']['coverage']["total_area"]).describe(),
                    "percentage_covered": pd.Series(all_result[name]['manual_data']['coverage']["percentage_covered"]).describe(),
                }
            }
    
    return final_result, all_result

def evaluate_replay(path_name, filename):
    '''
    path_name: <class str> -> Directory Path to get to file
    filename: <class str> -> File name to open

    return <class 'dict'> {
        "my_name": <class 'str'> -> name of robot
        "controller": <class 'str'> -> name of robot controller
        "finished": <class 'bool'> -> boolean if robot finished
        "displacement": <class 'numpy.float64'> -> total distance between the start and end position
        "total_distance": <class 'numpy.float64'> -> total distance robot traveled
        "obstructions": <class 'int'> -> number of times the robot was obstructed
        "my_completion_time": <class 'datetime.timedelta'> -> Time this robot finished
        "completion_time": <class 'datetime.timedelta'> -> Time all the robots finished
        "consensus": <class 'dict'> {
            "experimental_error": <class 'numpy.float64'> -> Error between calculated consesus point and actual ending point
            "min_distance": <class 'numpy.float64'> -> min distance to a neighbor
            "max_distance": <class 'numpy.float64'> -> max distance to a neighbor
            "adv_distance": <class 'numpy.float64'> -> adverage distance to a neighbor
        }
        "coverage": <class 'dict'> {
            "computed": <class 'bool'> -> determine if coverage was computed. If not, the other keys will not exist 
            "my_area": <class 'float'> -> area this robot covers
            "total_area": <class 'float'> -> total area in region
            "percentage_covered": <class 'numpy.float64'> -> percentage of the area this robot covers
        }
    }
    '''

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
                    robot_name = entry['my_name']
                    controller = entry['mainClass']
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
                    finished = True

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

    if not finished:
        print(f"File {filename} did not finish")

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
    '''
    path_name: <class str> -> Directory Path to get to file
    filename: <class str> -> File name to open

    return: <class 'list'> [
        <class 'list'> [
            <class 'dict'> {
                **Replay data from agent.py**
            }
        ]
    ]
    '''

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
    '''
    start_position: <class 'numpy,ndarray'> -> np.array([0,0])
    neighbor_start_position: <class 'dict'> -> {
        '1': <class 'numpy.ndarray'> -> np.array([0,0]),
        '2': <class 'numpy.ndarray'> -> np.array([0,0]),
        ...
    }
    final_position: <class 'numpy,ndarray'> -> np.array([10,10])
    neighbor_final_position: <class 'dict'> -> {
        '1': <class 'numpy.ndarray'> -> np.array([0,0]),
        '2': <class 'numpy.ndarray'> -> np.array([0,0]),
        ...
    }

    return <class 'dict'> {
        "experimental_error": <class 'numpy.float64'> -> Error between calculated consesus point and actual ending point
        "min_distance": <class 'numpy.float64'> -> min distance to a neighbor
        "max_distance": <class 'numpy.float64'> -> max distance to a neighbor
        "adv_distance": <class 'numpy.float64'> -> adverage distance to a neighbor
    }
    '''

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
    '''
    vor: <class 'scipy.spatial._qhull.Voroni'> -> vor = Voronoi([[0,0],[0,1],[1,1],[1,0]])
    point_idx: <class 'int'> -> the index of the point you want to evaluate
    bounding_shape: <class 'shapely.geometry.polygon.Polygon'> -> Polygon([[0,0],[0,-1],[-1,-1],[-1,0]])
    far_enough: <class 'float'> -> a value to stop the line stretching to inf

    return: <class 'shapely.geometry.polygon.Polygon'> -> intersection of my robot area and the boundary area
    '''
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
    '''
    my_position: <class 'numpy.ndarray'> -> np.array([0,0])
    neighbor_position: <class 'dict'> -> {
        '1': <class 'numpy.ndarray'> -> np.array([0,0]),
        '2': <class 'numpy.ndarray'> -> np.array([0,0]),
        ...
    }

    return <class 'dict'> {
        "computed": <class 'bool'> -> determine if coverage was computed. If not, the other keys will not exist 
        "my_area": <class 'float'> -> area this robot covers
        "total_area": <class 'float'> -> total area in region
        "percentage_covered": <class 'numpy.float64'> -> percentage of the area this robot covers
    }
    '''

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