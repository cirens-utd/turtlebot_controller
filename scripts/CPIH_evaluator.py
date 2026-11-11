#!/usr/bin/env python3

from pathlib import Path
from collections import defaultdict
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
#             └── Normal / SelfTrust
#                 └── Formation
#                     └── Trial
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
        Consensus Distance:
            - The total distance each robot is away from the good neighbors
            - Measures how effectective the convergance is

        All-Good Consensus Delta:
            - Final convergence point against theoretical consensus if every robot were honest. 
            - Measures how much the algorithm preserves teh swarm's original intent
        
        Good-Only Consensus Delta:
            - Final convergence point agasint theoritical consens of only honest robots
            - Measures adversary rejection effectiveness

        Correct Convergence
            - Determin whether the swarm converged to an acceptable location
            - Grade is based off if the swarm converges and where the swarm converges
                - 0 -> Didn't converge 
                - 0 -> Converged at adversaries
                - 1 -> Converged outside convey hull but away from adversaries
                - 2 -> Converged outside convex hull of good robots
            
        Wondering Time
            - The total amount of time a robot spends ouside teh initial good-agent convex hull.
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


        Parameters
        ----------
        experiement_data : dict
            {
                "trial_path": pathlib.PosixPath,
                "adversary_pattern": str,
                "graph_type": str,
                "push_mode": str,
                "mode": str,
                "formation": str,
                "trial_name": str,
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
            "trial_name": trial_path.name,
            "num_replay_files": len(replay_files),
            ß
            "adversary_pattern": adversary_pattern,
            "graph_type": graph_type,
            "push_mode": push_mode,
            "mode": mode,
            "formation": formation,

            # Grading Metrics
            "consensus_distance": None,
            "all_good_consensus_delta": None,
            "good_only_consensus_delta": None,
            "correct_convergance": None,
            "wondering_time": None,
            "safe_region_leakage": None,
            "safe_region_converage": None,
            "convergence_time": None,
        }
    """

    pdb.set_trace()

    return {
        "trial_name": trial_path.name,
        "num_replay_files": len(replay_files),

        "adversary_pattern": adversary_pattern,
        "graph_type": graph_type,
        "push_mode": push_mode,
        "mode": mode,
        "formation": formation,

        # Future metrics go here
        "all_good_error": None,
        "good_only_error": None,
        "converged": None,
        "outside_hull_percent": None,
        "safe_area_score": None,
        "convergence_time": None,
    }


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
            "adversary_pattern": rel_parts[0],
            "graph_type": rel_parts[1],
            "push_mode": rel_parts[2],
            "mode": rel_parts[3],
            "formation": rel_parts[4],
            "trial_name": rel_parts[5],
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
            [experiment_data["adversary_pattern"]] \
            [experiment_data["graph_type"]] \
            [experiment_data["push_mode"]] \
            [experiment_data["mode"]] \
            [experiment_data["formation"]] \
            [experiment_data["trial_name"]] = trial_result

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