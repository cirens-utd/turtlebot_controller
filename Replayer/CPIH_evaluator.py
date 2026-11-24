#!/usr/bin/env python3

from pathlib import Path
from collections import defaultdict
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, Point
from itertools import combinations
from datetime import datetime
from math import sqrt
import traceback
import copy
import json
import zipfile
import pdb


###############################################################################
# CONFIGURATION
###############################################################################

# Expected directory structure:
#
# Root
# └── Adversary Motion Pattern
#     └── Underlying Graph
#         └── Push / NonPush
#             └── Algorithm
#                 └── Normal / SelfTrust
#                     └── Formation
#                         ├── robot_1.turtleReplay
#                         ├── robot_2.turtleReplay
#                         └── ...
#
# Adjust if your hierarchy changes.
#
EXPECTED_DEPTH = 6

DEFAULT_ADVERSARY_CONTROLLERS = [
    "DONOTHING",
    "DORANDOM",
    "AdvBehavior",
]


###############################################################################
# EVALUATION
###############################################################################

def evaluate_trial(experiement_data):
    """
        Evaluates the Experiements and returns the results.

        ****************************************************
        ****************   Grading Merics   ****************
        ****************************************************

        #### Good Robots
        Consensus Distance:
            - The average distance each good robot is away from the good neighbors
            - Measures how effectective the convergance is

        All-Good Consensus Delta:
            - Final convergence point against theoretical consensus if every robot were honest. 
            - Measures how much the algorithm preserves teh swarm's original intent
        
        Good-Only Consensus Delta:
            - Final convergence point agasint theoritical consens of only honest robots
            - Measures adversary rejection effectiveness

        Hull-seperation Distance:
            - Find out how far away from the hull that we ended up
            - Measures the distance this robot is from the Hull

        Nearest Adversary Seperation:
            - Shows how close the swarm got to an adversary
            - Measures the distance from the swarm converagance to the nearest Adversary

        Average Adversary Seperation (Adversary Spread):
            - shows how close the swarm was to all the advesaires
            - Measures the distance to each adversary and finds the mean of these values.

        Farthest Adversary Seperation:
            - Shows how close the swarm got to the farthest adversary
            - Measures the distance from the swarm converagance to the farthest Adversary

        Correct Convergence
            - Determin whether the swarm converged to an acceptable location
            - Grade is based off if the swarm converges and where the swarm converges
                - 0 -> Didn't converge 
                - 1 -> Converged at adversaries
                - 2 -> Converged outside convey hull but away from adversaries
                - 3 -> Converged inside convex hull of good robots
            
        Wondering Time
            - The total amount of time a robot spends ouside the initial good-agent convex hull.
            - The reference hull is generated from the good robots' position at time 0s
            - Measures the swarm's deviation from the original safe operation region

        Safe Region Leakage
            - Computes the amount of the calculated region from each robot leaking out of the convex hull of goood robots at each iteration
            - Measures the probability of selecting an unsafe point in each iteration

        Safe Region Converage
            - Computes how much of the convex hull the region was able to cover
            - Measures the percent of the calculated region covers the convex hull region in each iteration

        Convergence Time
            - Experiement Completion Time
            - Time from each robot going to Running to Complete or cancelation of the code
            - Measures the time used in each experiement

        Maximum Tukey Depth
            - Shows the maximum tukey depth seen in the run

        Average Tukey Depth
            - Takes an average of the tukey depth over the time the robot was running

        Minimum Tukey Depth
            - Shows the minimum tukey depth seen in the run

        Centerpoint Depth
            - Shows the centerpoint depth for the run

        #### Adversary Robots
        Adversary Footprint:
            - The size of the convex hull of the adversaries showing the area the adversaiers make together
            - Measures the convex hull between the adversaries
            
        Consensus Distance:
            - The average distance each adversary robot is away from the other adversary neighbors
            - Measures how effectective the convergance is

        Consensus Delta:
            - the total distance from adversaries converages to the center of the convex hull of good robots
            - measures how far away the adversaries were from the center of the good robots convex hull

        Nearest Adversary Seperation:
            - Shows how close the swarm got to an adversary
            - Measures the distance from the swarm converagance to the nearest Adversary

        Average Adversary Delta (Adversary Spread):
            - shows how close the swarm was to all the advesaires
            - Measures the distance to each adversary and finds the mean of these values.

        Farthest Adversary Seperation:
            - Shows how close the swarm got to the farthest adversary
            - Measures the distance from the swarm converagance to the farthest Adversary

        Parameters
        ----------
        experiement_data : dict
            {
                "trial_path": pathlib.PosixPath,
                "adversary_motion": str,
                "graph_type": str,
                "push_mode": str,
                "alorithm": str,
                "mode": str,
                "formation": str,
                "replay_files": list[pathlib.Path],
                "robots": dict
                    {
                        "robots": {
                            robot_name: {
                                "filename": str,
                                "controller": str,
                                "is_adversary": bool,
                                "samples": list,
                                "initial_state": dict,
                                "final_state": dict
                            }
                        },
                        "num_robots": int,
                        "num_adversaries": int,
                        "num_good_robots": int,
                        "adversary_names": list,
                        "good_robot_names": list
                    }
            }

        Returns
        -------
        dict
        {
            # Trial Information
            "trial_name": trial_name,
            "trial_path": trial_path,
            "num_replay_files": len(replay_files),
            
            # Trial Parameters
            "adversary_motion": adversary_motion,
            "graph_type": graph_type,
            "push_mode": push_mode,
            "algorithm": algorithm,
            "mode": mode,
            "formation": formation,

            # Grading Metrics
            "Good_Results" : {
                "Robot1": {
                    "consensus_distance": None,
                    "all_good_consensus_delta": None,
                    "good_only_consensus_delta": None,
                    hull_seperation_distance": None,
                    "nearest_adversary_seperation": None,
                    "average_adversary_seperation": None,
                    "farthest_adversary_seperation": None,
                    "correct_convergance": None,
                    "wondering_time": None,
                    "safe_region_leakage": None,
                    "safe_region_converage": None,
                    "convergence_time": None,
                    "maximum_tukey_depth": None,
                    "average_tukey_depth": None,
                    "minimum_tukey_depth": None,
                    "centerpoint_depth": None
                },
                "Robot2": {}...
            },
            "Bad_results": {
                "Robot10": {
                    "adversary_footprint": None,
                    "consensus_distance": None,
                    "consensus_delta": None,
                    "nearest_adversay_delta": None,
                    "average_adversary_delta": None,
                    "farthest_adversary_delta": None
                },
                "Robot11": ...
            }
        }
    """
    sep = '_'
    trial_name = experiement_data['adversary_motion'] + sep + \
        experiement_data['graph_type'] + sep + \
        experiement_data['push_mode'] + sep + \
        experiement_data['algorithm'] + sep + \
        experiement_data['mode'] + sep + \
        experiement_data['formation']

    adversary_names = experiement_data['robots']['adversary_names']

    good_results = {}
    bad_results = {}

    robots = experiement_data['robots']['robots']

    # Loop Through each robot
    for robot_name, robot_data in robots.items():
        samples = robot_data['samples']

        if robot_data['is_adversary']:
            bad_results[robot_name] = compute_adversary_metrics(samples, adversary_names)
        else:
            good_results[robot_name] = compute_good_metrics(samples, adversary_names)

    return {
        # Trial Information
        "trial_name": trial_name,
        "trial_path": experiement_data['trial_path'],
        "num_replay_files": len(experiement_data['replay_files']),
        
        # Trial Parameters
        "adversary_motion": experiement_data['adversary_motion'],
        "graph_type": experiement_data['graph_type'],
        "push_mode": experiement_data['push_mode'],
        "algorithm": experiement_data['algorithm'],
        "mode": experiement_data['mode'],
        "formation": experiement_data['formation'],

        "Good_Results" : good_results.copy(),
        "Bad_results": bad_results.copy()
    }

def compute_adversary_metrics(samples, adversary_names):
    """
        #### Adversary Robots
        Adversary Footprint:
            - The size of the convex hull of the adversaries showing the area the adversaiers make together
            - Measures the convex hull between the adversaries
            
        Consensus Distance:
            - The average distance each adversary robot is away from the other adversary neighbors
            - Measures how effectective the convergance is

        Consensus Delta:
            - the total distance from adversaries converages to the center of the convex hull of good robots
            - measures how far away the adversaries were from the center of the good robots convex hull

        Nearest Adversary Seperation:
            - Shows how close the swarm got to an adversary
            - Measures the distance from the swarm converagance to the nearest Adversary

        Average Adversary Delta (Adversary Spread):
            - shows how close the swarm was to all the advesaires
            - Measures the distance to each adversary and finds the mean of these values.

        Farthest Adversary Seperation:
            - Shows how close the swarm got to the farthest adversary
            - Measures the distance from the swarm converagance to the farthest Adversary

        Parameters
        ----------
        "samples": list
            samples.keys() = ['time', 'my_name', 'mainClass', 'replayVersion', 'robot_status', 'robot_ready', 'position_started', 'neighbors_started', 'lidar_started', 'camera_started', 'camera_setup', 'battery_received', 'wait_for_battery', 'robot_moving', 'robot_moving_wait', 'desired_heading', 'destination_reached', 'motion_complete', 'neighbors_complete', 'movement_restricted', 'led_light_state', 'battery_dict', 'path_obstructed', 'path_obstructed_laser', 'path_obstructed_neighbor', 'laser_avoid_error', 'destination_tolerance', 'angle_tolerance', 'desired_location', 'attempted_desired_location', 'desired_angle', 'my_pose', 'neighbor_poses', 'neighbor_position', 'neighbor_orientation', 'safe_area', 'tukey_depth', 'center_depth', 'self_trust', 'safe_point_mode', 'imprecision']

        Returns
        -------
        dict
        {
            "adversary_footprint": None,
            "consensus_distance": None,
            "consensus_delta": None,
            "nearest_adversay_delta": None,
            "average_adversary_delta": None,
            "farthest_adversary_delta": None
        }
    """
    adversary_results_template = {
        "adversary_footprint": None,
        "consensus_distance": 0.0,
        "consensus_delta": None,
        "nearest_adversay_delta": None,
        "average_adversary_delta": None,
        "farthest_adversary_delta": None
    }

    if not samples:
        return adversary_results_template.copy()

    final_sample = samples[-1]

    my_position = [
                        final_sample['my_pose']['pose']['position']['x'],
                        final_sample['my_pose']['pose']['position']['y']
                    ]

    good_positions = []
    bad_positions = [my_position]

    for robot_id, pose_data in final_sample["neighbor_poses"].items():

        robot_name = f"robot{robot_id}"
        position  = [
                        pose_data['pose']['position']['x'],
                        pose_data['pose']['position']['y']
                    ]

        if robot_name in adversary_names:
            bad_positions.append(position)

        else:
            good_positions.append(position)

    # Adversary Footprint
    bad_hull = build_convex_hull(bad_positions)

    adversary_results_template['adversary_footprint'] = (
        bad_hull.area
        if bad_hull is not None
        else 0.0
    )

    # Consensus Distance
    other_adversaries = [position for position in bad_positions if position != my_position]

    if other_adversaries:
        adversary_results_template['consensus_distance'] = (
            sum(
                distance(my_position, position)
                for position in other_adversaries
            ) / len(other_adversaries)
        )

    # Consensus Delta
    if bad_positions:
        bad_consensus = mean_position(bad_positions)
    else:
        bad_consensus = my_position

    good_hull = build_convex_hull(good_positions)

    if good_hull is not None:
        good_centroid = [good_hull.centroid.x, good_hull.centroid.y]
    else:
        good_centroid = mean_position(good_positions)

    adversary_results_template['consensus_delta'] = distance(bad_consensus, good_centroid)

    # Adversary Delta Metrics
    if good_positions:
        good_distances = [
            distance(my_position, position)
            for position in good_positions
        ]

        adversary_results_template['nearest_adversary_delta'] = min(good_distances)

        adversary_results_template['average_adversary_delta'] = (
            sum(good_distances)
            / len(good_distances)
        )

        adversary_results_template['farthest_adversary_delta'] = max(good_distances)

    else:

        nearest_adversary_delta = None
        average_adversary_delta = None
        farthest_adversary_delta = None

    return adversary_results_template.copy()


def compute_good_metrics(samples, adversary_names = []):
    """
        #### Good Robots
        Consensus Distance:
            - The average distance each good robot is away from the good neighbors
            - Measures how effectective the convergance is

        All-Good Consensus Delta:
            - Final convergence point against theoretical consensus if every robot were honest. 
            - Measures how much the algorithm preserves teh swarm's original intent
        
        Good-Only Consensus Delta:
            - Final convergence point agasint theoritical consens of only honest robots
            - Measures adversary rejection effectiveness

        Nearest Adversary Seperation:
            - Shows how close the swarm got to an adversary
            - Measures the distance from the swarm converagance to the nearest Adversary

        Average Adversary Seperation (Adversary Spread):
            - shows how close the swarm was to all the advesaires
            - Measures the distance to each adversary and finds the mean of these values.

        Farthest Adversary Seperation:
            - Shows how close the swarm got to the farthest adversary
            - Measures the distance from the swarm converagance to the farthest Adversary

        Correct Convergence
            - Determin whether the swarm converged to an acceptable location
            - Grade is based off if the swarm converges and where the swarm converges
                - 0 -> Didn't converge 
                - 1 -> Converged at adversaries
                - 2 -> Converged outside convey hull but away from adversaries
                - 3 -> Converged inside convex hull of good robots
            
        Wondering Time
            - The total amount of time a robot spends ouside the initial good-agent convex hull.
            - The reference hull is generated from the good robots' position at time 0s
            - Measures the swarm's deviation from the original safe operation region

        Safe Region Leakage
            - Computes the amount of the calculated region from each robot leaking out of the convex hull of goood robots at each iteration
            - Measures the probability of selecting an unsafe point in each iteration

        Safe Region Converage
            - Computes how much of the convex hull the region was able to cover
            - Measures the percent of the calculated region covers the convex hull region in each iteration

        Convergence Time
            - Experiement Completion Time
            - Time from each robot going to Running to Complete or cancelation of the code
            - Measures the time used in each experiement

        Maximum Tukey Depth
            - Shows the maximum tukey depth seen in the run

        Average Tukey Depth
            - Takes an average of the tukey depth over the time the robot was running

        Minimum Tukey Depth
            - Shows the minimum tukey depth seen in the run

        Centerpoint Depth
            - Shows the centerpoint depth for the run

        Parameters
        ----------
        "samples": list
            samples.keys() = ['time', 'my_name', 'mainClass', 'replayVersion', 'robot_status', 'robot_ready', 'position_started', 'neighbors_started', 'lidar_started', 'camera_started', 'camera_setup', 'battery_received', 'wait_for_battery', 'robot_moving', 'robot_moving_wait', 'desired_heading', 'destination_reached', 'motion_complete', 'neighbors_complete', 'movement_restricted', 'led_light_state', 'battery_dict', 'path_obstructed', 'path_obstructed_laser', 'path_obstructed_neighbor', 'laser_avoid_error', 'destination_tolerance', 'angle_tolerance', 'desired_location', 'attempted_desired_location', 'desired_angle', 'my_pose', 'neighbor_poses', 'neighbor_position', 'neighbor_orientation', 'safe_area', 'tukey_depth', 'center_depth', 'self_trust', 'safe_point_mode', 'imprecision']


        Returns
        -------
        dict
        {
            "consensus_distance": None,
            "all_good_consensus_delta": None,
            "good_only_consensus_delta": None,
            "hull_seperation_distance": None,
            "nearest_adversary_seperation": None,
            "average_adversary_seperation": None,
            "farthest_adversary_seperation": None,
            "correct_convergance": None,
            "wondering_time": None,
            "safe_region_leakage": None,
            "safe_region_converage": None,
            "convergence_time": None,
            "maximum_tukey_depth": None,
            "average_tukey_depth": None,
            "minimum_tukey_depth": None,
            "centerpoint_depth": None
        }
    """

    good_results_template = {
        # Good Grading Metrics
        "consensus_distance": None,
        "all_good_consensus_delta": None,
        "good_only_consensus_delta": None,
        "hull_seperation_distance": None,
        "nearest_adversary_seperation": float("inf"),
        "average_adversary_seperation": None,
        "farthest_adversary_seperation": float("-inf"),
        "correct_convergance": None,
        "wondering_time": 0,
        "safe_region_leakage": None,
        "safe_region_converage": None,
        "convergence_time": None,
        "maximum_tukey_depth": float("-inf"),
        "average_tukey_depth": None,
        "minimum_tukey_depth": float("inf"),
        "centerpoint_depth": None
    }

    # variables for calculations
    started_running = False
    start_time = None
    end_time = None
    good_start = []
    bad_start = []
    good_end = []
    bad_end = []
    good_hull = None
    tukey_sum = 0
    tukey_count = 0
    leakage_sum = 0
    leakage_count = 0
    coverage_sum = 0
    coverage_count = 0
    previous_sample = None

    for sample in samples:

        status = sample["robot_status"]

        if not started_running:

            if status != "MOVING":
                continue

            started_running = True
            start_time = parse_time(sample["time"])

            my_position = [
                            sample['my_pose']['pose']['position']['x'],
                            sample['my_pose']['pose']['position']['y']
                        ]

            # Build hull on first iteration
            good_start = [my_position]

            for other_id, other_data in sample["neighbor_poses"].items():

                robot_id = f"robot{other_id}"
                pos  = [
                            other_data['pose']['position']['x'],
                            other_data['pose']['position']['y']
                        ]

                if robot_id in adversary_names:
                    bad_start.append(pos)
                    continue
                good_start.append(pos)

            good_hull = build_convex_hull(good_start)


        my_position = [
                        sample['my_pose']['pose']['position']['x'],
                        sample['my_pose']['pose']['position']['y']
                    ]

        # Wandering Time
        if previous_sample is not None:
            dt = (
                parse_time(sample["time"])
                - parse_time(previous_sample["time"])
            ).total_seconds()

            if (good_hull is not None and not good_hull.contains(Point(my_position))):
                good_results_template['wondering_time'] += dt

        # Tukey Metrics
        depth = sample["tukey_depth"]
        good_results_template['maximum_tukey_depth'] = max(good_results_template['maximum_tukey_depth'], depth)
        good_results_template['minimum_tukey_depth'] = min(good_results_template['minimum_tukey_depth'], depth)
        tukey_sum += depth
        tukey_count += 1

        # Center Depth
        good_results_template['centerpoint_depth'] = sample["center_depth"]

        # Safe Region Metrics

        safe_area = sample["safe_area"]

        if len(safe_area) >= 3:

            safe_polygon = Polygon(safe_area)

            if good_hull is not None and safe_polygon.is_valid:
                leak_area = (safe_polygon.difference(good_hull).area)
                leakage_sum += (leak_area / safe_polygon.area)
                leakage_count += 1
                coverage_sum += (safe_polygon.intersection(good_hull).area / good_hull.area)
                coverage_count += 1
            else:
                print("WHY ARE WE HERE???")
                # pdb.set_trace()

        if status == "COMPLETE":
            end_time = parse_time(sample["time"])

        previous_sample = sample

    if status != "COMPLETE":
        end_time = parse_time(sample["time"])

    # Concensus Distance
    concensus_distances = []
    good_end = [my_position]

    for other_id, other_data in sample["neighbor_poses"].items():
        robot_id = f"robot{other_id}"
        pos = [
                    other_data['pose']['position']['x'],
                    other_data['pose']['position']['y']
                ]

        if robot_id in adversary_names:
            bad_end.append(pos)
            continue
        good_end.append(pos)
        concensus_distances.append(distance(my_position, pos))


    good_results_template['consensus_distance'] = sum(concensus_distances) / len(concensus_distances)
    final_consensus_point = mean_position(good_end)

    # Concensus deltas
    all_good_consensus_point = mean_position(good_start + bad_start)
    good_only_consensus_point = mean_position(good_start)
    good_results_template['hull_seperation_distance'] = good_hull.distance(Point(my_position))
    good_results_template['all_good_consensus_delta'] = distance(final_consensus_point, all_good_consensus_point)
    good_results_template['good_only_consensus_delta'] = distance(final_consensus_point, good_only_consensus_point)

    # Adversary Calculations
    nearest_distance = float('inf')
    farthest_distance = 0
    distance_sum = 0
    for pose in bad_end:
        d = distance(final_consensus_point, pose)

        nearest_distance = min(nearest_distance, d)
        farthest_distance = max(farthest_distance, d)
        distance_sum += d
    good_results_template['nearest_adversary_seperation'] = nearest_distance
    good_results_template['average_adversary_seperation'] = distance_sum / len(bad_end)
    good_results_template['farthest_adversary_seperation'] = farthest_distance

    # Final calculations
    good_results_template['correct_convergance'] = grade_convergance(good_end, bad_end, good_hull)
    good_results_template['average_tukey_depth'] = (tukey_sum / tukey_count if tukey_count else None)
    good_results_template['safe_region_leakage'] = (leakage_sum / leakage_count if leakage_count else None)
    good_results_template['safe_region_converage'] = (coverage_sum / coverage_count if coverage_count else None)
    good_results_template['convergence_time'] = ((end_time - start_time).total_seconds() if start_time and end_time else None)

    return good_results_template.copy()

def grade_convergance(good, bad, hull, convergence_threshold=0.5, adversary_threshold=1.0):
    """
    Returns:
        0 -> Did not converge
        1 -> Converged at adversaries
        2 -> Converged outside convex hull of good robots
        3 -> Converged inside convex hull of good robots
    """

    # Compute convergence point
    # (centroid of final good robot positions)

    convergence_point = [
        sum(p[0] for p in good) / len(good),
        sum(p[1] for p in good) / len(good)
    ]

    # Check convergence
    max_robot_seperation = 0.0
    for p1, p2 in combinations(good, 2):
        max_robot_seperation = max(max_robot_seperation, distance(p1, p2))

    if max_robot_seperation > convergence_threshold:
        return 0

    # Check whether convergence point
    # lies inside the final good hull

    if good_hull.contains(Point(convergence_point)):
        return 3

    #
    # Check proximity to adversaries
    #

    nearest_adversary = min(distance(adv, convergence_point) for adv in adversary_positions)

    if nearest_adversary < adversary_threshold:
        return 1

    return 2

###############################################################################
# Utility Function
###############################################################################
def parse_time(timestamp):
    """
        timestamp: 2026-10-23.100144
        Formate is YYYY-MM-DD.HHMMSS
    """
    return datetime.strptime( timestamp, "%Y-%m-%d.%H%M%S")

def distance(p1, p2):
    """
        Find the distance between two points
        p1: [x1, y1]
        p2: [x2, y2]

        return sqrt( (x1 - x2)^2 + (y1 - y2)^2 )
    """
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def mean_position(points):
    """
        Finds the mean of all the points provided
        points: [
                    [x1, y1],
                    [x2, y2]
                ]
    """
    return [
                sum(p[0] for p in points) / len(points),
                sum(p[1] for p in points) / len(points)
            ]
def build_convex_hull(points):
    """
        points = [
            [x1, y1],
            [x2, y2],
            ...
        ]

        Given a set of points, will return a shapely polygon of the hull
    """

    if len(points) < 3:
        return None

    try:

        hull = ConvexHull(points)

        hull_points = [
            points[i]
            for i in hull.vertices
        ]

        return Polygon(hull_points)

    except Exception as error:
        return None

###############################################################################
# RESULT STORAGE
###############################################################################

def nested_dict():
    return defaultdict(nested_dict)


###############################################################################
# EXPERIMENT DISCOVERY
###############################################################################

def discover_experiments(root_path):
    root_path = Path(root_path)

    hierarchy_results = nested_dict()
    flat_results = []

    processed_trials = set()

    for replay_file in root_path.rglob("*.turtleReplay"):

        trial_dir = replay_file.parent

        if trial_dir in processed_trials:
            continue

        processed_trials.add(trial_dir)

        rel_parts = trial_dir.relative_to(root_path).parts

        if len(rel_parts) < EXPECTED_DEPTH:
            continue

        experiment_data = {
            "trial_path": trial_dir,
            "adversary_motion": rel_parts[0],
            "graph_type": rel_parts[1],
            "push_mode": rel_parts[2],
            "algorithm": rel_parts[3],
            "mode": rel_parts[4],
            "formation": rel_parts[5],
            "replay_files": sorted(
                trial_dir.glob("*.turtleReplay")
            ),
        }

        experiment_data["robots"] = load_trial_data(
            experiment_data["replay_files"]
        )

        trial_result = evaluate_trial(experiment_data)

        flat_results.append(trial_result)

        hierarchy_results \
            [experiment_data["adversary_motion"]] \
            [experiment_data["graph_type"]] \
            [experiment_data["push_mode"]] \
            [experiment_data["algorithm"]] \
            [experiment_data["mode"]] \
            [experiment_data["formation"]] = trial_result

    return hierarchy_results, flat_results

def load_trial_data(replay_files, adversary_controllers=None):
    """
    Loads all replay files associated with a trial.

    Parameters
    ----------
    replay_files : list[pathlib.Path]

    adversary_controllers : list[str], optional

    Returns
    -------
    dict

    {
        "robots": {
            robot_name: {
                "filename": str,
                "controller": str,
                "is_adversary": bool,
                "samples": list,
                "initial_state": dict,
                "final_state": dict
            }
        },

        "num_robots": int,
        "num_adversaries": int,
        "num_good_robots": int,
        "adversary_names": list,
        "good_robot_names": list
    }
    """

    if adversary_controllers is None:
        adversary_controllers = DEFAULT_ADVERSARY_CONTROLLERS
    adversary_set = {
        controller.lower()
        for controller in adversary_controllers
    }

    robots = {}

    for replay_file in replay_files:

        replay_file = Path(replay_file)

        replay_data = []

        try:
            with zipfile.ZipFile(replay_file, "r") as zip_ref:
                inner_file = zip_ref.namelist()[0]

                with zip_ref.open(inner_file) as curFile:
                    for line in curFile:
                        line = line.decode("utf-8", errors="ignore").strip()

                        if not line:
                            continue

                        replay_data.extend(json.loads(line))

        except Exception as ex:
            print(f"Failed loading {replay_file}: {ex}")
            continue

        if len(replay_data) == 0:
            continue

        first_entry = replay_data[0]
        last_entry = replay_data[-1]

        robot_name = first_entry.get(
            "my_name",
            replay_file.stem
        )

        controller = first_entry.get("mainClass")

        #
        # Attempt to determine adversary status.
        #
        is_adversary = (
            controller is not None
            and controller.lower() in adversary_set
        )


        robots[robot_name] = {
            "filename": replay_file.name,
            "controller": controller,
            "is_adversary": is_adversary,
            "samples": replay_data,
            "initial_state": first_entry,
            "final_state": last_entry,
        }

    #########################
    # Trial-level statistics#
    #########################

    adversary_names = [
        name
        for name, robot in robots.items()
        if robot["is_adversary"]
    ]

    good_robot_names = [
        name
        for name, robot in robots.items()
        if not robot["is_adversary"]
    ]

    return {
        "robots": robots,
        "num_robots": len(robots),
        "num_adversaries": len(adversary_names),
        "num_good_robots": len(good_robot_names),
        "adversary_names": adversary_names,
        "good_robot_names": good_robot_names
    }


###############################################################################
# UTILITY
###############################################################################

def defaultdict_to_dict(d):
    """
    Converts nested defaultdicts into normal dictionaries
    for JSON export.
    """

    if isinstance(d, defaultdict):
        d = {k: defaultdict_to_dict(v) for k, v in d.items()}
    return d


###############################################################################
# MAIN
###############################################################################

if __name__ == "__main__":

    root = Path("/Users/jss150630/Programs/VM/UbuntuJellyfish/SharedDrive/turtlebot_simulator/Replays/CPIH_Paper/Experiments")
    output_dir = root.parent

    hierarchy_results, flat_results = discover_experiments(root)

    hierarchy_results = defaultdict_to_dict(hierarchy_results)

    with open(output_dir / "hierarchical_results.json", "w") as fp:
        json.dump(hierarchy_results, fp, indent=4)

    with open(output_dir / "flat_results.json", "w") as fp:
        json.dump(flat_results, fp, indent=4)

    print(f"Trials Processed: {len(flat_results)}")