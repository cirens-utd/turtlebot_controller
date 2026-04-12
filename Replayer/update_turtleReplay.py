import json
import zipfile
import os
from copy import deepcopy
from os import listdir, remove, rmdir, path, getcwd
import tkinter as tk
from tkinter import filedialog
import pdb


# =========================
# Latest Schema Definition
# =========================
LATEST_SCHEMA = {
    "time": "",
    "my_name": "",
    "mainClass": "",
    "replayVersion": 1, 

    # Robot Conditions
    "robot_status": None,
    "robot_ready": False,
    "position_started": False,
    "neighbors_started": False,
    "lidar_started": False,
    "camera_started": False,
    "camera_setup": False,
    "battery_received": False,
    "wait_for_battery": False,
    "robot_moving": False,
    "desired_heading": None,
    "destination_reached": False,
    "motion_complete": False,
    "neighbors_complete": False,
    "movement_restricted": False,

    # LED Info
    "led_light_state": {},

    # Battery Info
    "battery_dict": {
            "voltage": None,
            "temperature": None,
            "current": None,
            "charge": None,
            "capacity": None,
            "design_capacity": None,
            "percentage": None,
            "power_supply_status": None,
            "power_supply_health": None,
            "power_supply_technology": None,
            "present": None,
            "cell_voltage": [],
            "cell_temperature": [],
            "location": '',
            "serial_number": ''
    },

    # Avoidance Conditions
    "path_obstructed": False,
    "path_obstructed_laser": False,
    "path_obstructed_neighbor": False,
    "laser_avoid_error": None,

    # Goal Info
    "destination_tolerance": None,
    "angle_tolerance": None,
    "desired_location": None,
    "attempted_desired_location": None,
    "desired_angle": None,

    # Tracking positions
    "my_pose": {
            "header": {
                "stamp": {
                    "sec": 0,
                    "nanosec": 0
                },
                "frame_id": 0
            },
            "pose": {
                "position": {},
                "orientation": {}
            }
        },
    "neighbor_poses": {
        "__dynamic__":{
            "header": {
                "stamp": {
                    "sec": 0,
                    "nanosec": 0
                },
                "frame_id": 0
            },
            "pose": {
                "position": {},
                "orientation": {}
            },
            "in_neighborhood": True
        }
    }
}

uncompress_file = None


# =========================
# Schema Fix Function
# =========================
def fill_missing(data, template, path="", start=True, updated_errors=None):
    """
    Recursively fills missing keys in 'data' using 'template'.
    Does NOT overwrite existing values.
    """
    if not updated_errors:
        updated_errors = set()

    for key, value in template.items():
        full_key = f"{path}.{key}" if path else key

        if start:
            for line_data in data:
                updated_errors = _fill_missing_function(key, full_key, value, line_data, updated_errors)
        else:
            updated_errors = _fill_missing_function(key, full_key, value, data, updated_errors)

    return data, updated_errors

def _fill_missing_function(key, full_key, value, data, updated_errors):
    if key not in data:
        if full_key not in updated_errors:
                updated_errors.add(full_key)
        data[key] = deepcopy(value)

    elif isinstance(value, dict) and "__dynamic__" in value and isinstance(data[key], dict):
            template = value["__dynamic__"]

            for sub_key, sub_value in data[key].items():
                if isinstance(sub_value, dict):
                    _, updated_errors = fill_missing(
                        sub_value,
                        template,
                        path=f"{full_key}.{sub_key}",
                        start=False,
                        updated_errors=updated_errors
                    )
    elif isinstance(value, dict) and isinstance(data[key], dict):
        _, updated_errors = fill_missing(data[key], value, full_key + "." + key, False, updated_errors)

    return updated_errors

# =========================
# Load Replay from Zip
# =========================
def load_replay_zip(zip_path, extract_dir="usable_replay"):
    """
    Extracts a replay zip and loads JSON lines into a list.
    """
    global uncompress_file

    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_dir)

    file_name = os.listdir(extract_dir)[0]
    uncompress_file = file_name
    full_path = os.path.join(extract_dir, file_name)

    data = []

    with open(full_path, 'r', errors="ignore") as f:
        lines = f.read().strip().split("\n")
        for line in lines:
            data.append(json.loads(line))

    # Cleanup extracted files
    os.remove(full_path)
    os.rmdir(extract_dir)

    return data


# =========================
# Fix Entire Replay Dataset
# =========================
def fix_replay_data(data, verbose=False):
    """
    Applies schema fix to every entry in the replay data.
    """
    fixed_data = []
    updated_errors = set()
    for entry in data:
        fixed_entry, updated_errors = fill_missing(entry, LATEST_SCHEMA, updated_errors=updated_errors)
        fixed_data.append(fixed_entry)

    if verbose:
        if len(updated_errors):
            print(f"Updated Keys:")
            for key in updated_errors:
                if key.split(".")[-1] == "position" or key.split(".")[-1] == "orientation":
                    print(f"********Warnring*******")
                print(f"{key}")
        else:
            print("No Values Updated")

    return fixed_data


# =========================
# Save Replay (JSON lines)
# =========================
def save_replay(data, output_file=None, output_dir=None):
    """
    Saves replay data as JSON lines (same format as original).
    """
    global uncompress_file

    if not output_file:
        output_file = uncompress_file

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, output_file)

    with open(output_file, 'w') as f:
        for entry in data:
            f.write(json.dumps(entry) + "\n")

    return output_file


# =========================
# Save Replay as Zip
# =========================
def save_replay_zip(data, output_dir=None, zip_name=None, temp_file=None):
    """
    Saves replay data and compresses into a zip file.
    """
    global uncompress_file

    if not zip_name:
        zip_name = uncompress_file.split('.')[0] + ".turtleReplay"
    if not temp_file:
        temp_file = uncompress_file

    # Ensure directory exists
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        zip_path = os.path.join(output_dir, zip_name)
        temp_path = os.path.join(output_dir, temp_file)
    else:
        zip_path = zip_name
        temp_path = temp_file

    # Save temp JSON
    save_replay(data, temp_path)

    # Zip it
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        zipf.write(temp_path, arcname=os.path.basename(temp_path))

    # Remove temp file
    os.remove(temp_path)

    return zip_path


# =========================
# One-shot Utility Function
# =========================
def load_fix_and_save(zip_input, zip_output=None, output_dir=None, verbose=False, save=True):
    """
    Full pipeline:
    Load → Fix → Save (optionally zipped)
    """
    data = load_replay_zip(zip_input)
    fixed_data = fix_replay_data(data, verbose=verbose)

    if save:
        if zip_output:
            save_replay_zip(fixed_data, zip_name=zip_output, output_dir=output_dir)
        else:
            save_replay(fixed_data, output_dir=output_dir)

    return fixed_data

if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()

    # Default directory logic
    start_path = path.abspath(path.join(getcwd(), "..", "Replays"))
    if not path.exists(start_path):
        start_path = path.abspath(path.join(getcwd(), "Replays"))

    replay_files = filedialog.askopenfilenames(
        title="Select Replay file(s)",
        initialdir=start_path,
        filetypes=[("Replay Files", "*.turtleReplay")]
    )

    root.destroy()

    if not replay_files:
        print("No files selected.")
        exit()

    for file_path in replay_files:
        print(f"Processing: {file_path}")

        # Example: save into "fixed" subfolder next to original file
        # output_dir = path.join(path.dirname(file_path), "fixed")
        output_dir = path.dirname(file_path)

        load_fix_and_save(
            file_path,
            zip_output=path.basename(file_path),
            output_dir=output_dir,
            verbose=True
        )

    print("Done processing all files.")