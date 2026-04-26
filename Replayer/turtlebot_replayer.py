import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, Slider
import matplotlib.animation as animation
import numpy as np
from os import listdir, remove, rmdir, path, getcwd
from collections import defaultdict
import zipfile
import tkinter as tk
from tkinter import filedialog, messagebox
from update_turtleReplay import load_fix_and_save, LATEST_SCHEMA
import argparse
import pdb

class DotDict(defaultdict):
    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError:
            self[key] = self.default_factory()
            return self[key]

    def __setattr__(self, key, value):
        self[key] = value

class ReplayVisualizer:
    def __init__(self, play=True, save=False, filename='MyReplay', beautify=False, *args, trail_length=100):
        self.play = play
        self.save = save
        self.filename = filename
        self.beautify = beautify
        self.replay_data = []

        self.title = "Robot Position Over Time"
        self.paused = False
        self.size = (10,6)                  # (x, y) x inches wide and y inches tall
        self.trail_length = trail_length
        self.total_frames = 0
        self.frame_rate = 10
        self.xmin, self.xmax = -10, 10
        self.ymin, self.ymax = -10, 10
        self.fontsize = 12

        self.robot_radius = 0.4
        self._offgrid = (-100, -100)

        self.start_line_x = 0.02
        self.start_line_y = 0.95
        self.delta_line_x = 0.1
        self.delta_line_y = 0.05
        self.x_indent = 0.015

        self.plugins = []
        self.data = DotDict(list)
        self._setup_extract_maps()

    def _setup_extract_maps(self):
        self.replay_schema = {
            # --- robot state ---
            "robot_status",
            "robot_ready",
            "position_started",
            "neighbors_started",
            "lidar_started",
            "wait_for_battery",
            "battery_received",
            "robot_moving",

            # --- motion / planning ---
            "desired_heading",
            "destination_reached",
            "motion_complete",
            "neighbors_complete",

            # --- safety / avoidance ---
            "movement_restricted",
            "path_obstructed",
            "path_obstructed_laser",
            "path_obstructed_neighbor",
            "laser_avoid_error",

            # --- LED ---
            "led_light_state",

            # --- goal ---
            "destination_tolerance",
            "angle_tolerance",
            "desired_location",
            "attempted_desired_location",
            "desired_angle",

            # --- raw / structured data ---
            "battery_dict",
            "my_pose",
            "neighbor_poses",
            "neighbor_position"
        }

        self.extract_map = {
            "my_pose": self._extract_pose,
            "neighbor_poses": self._extract_neighbors,
        }

    def _extract_pose(self, entry):
        pose_msg = entry.get("my_pose")

        if pose_msg is None:
            self.data.x_vals.append(0)
            self.data.y_vals.append(0)
            self.data.yaws.append(0)
            return

        pose = pose_msg["pose"]

        x = pose["position"]["x"]
        y = pose["position"]["y"]

        ori = pose["orientation"]
        qx, qy, qz, qw = ori["x"], ori["y"], ori["z"], ori["w"]

        yaw = np.remainder(
            (np.arctan2(
                2 * (qw * qz + qx * qy),
                1 - 2 * (qy * qy + qz * qz)
            ) + np.pi),
            2 * np.pi
        )

        self.data.x_vals.append(x)
        self.data.y_vals.append(y)
        self.data.yaws.append(yaw)

    def _extract_neighbors(self, entry):
        neighbors_data = {}

        for name, pose in entry.get("neighbor_poses", {}).items():

            ori = pose["pose"]["orientation"]
            qx, qy, qz, qw = ori["x"], ori["y"], ori["z"], ori["w"]

            yaw = np.remainder(
                (np.arctan2(
                    2 * (qw * qz + qx * qy),
                    1 - 2 * (qy * qy + qz * qz)
                ) + np.pi),
                2 * np.pi
            )

            neighbors_data[name] = {
                "x": pose["pose"]["position"]["x"],
                "y": pose["pose"]["position"]["y"],
                "yaw": yaw,
                "in_neighborhood": pose["in_neighborhood"]
            }

        self.data.neighbor_poses.append(neighbors_data)

    def add_plugin(self, plugin):
        self.plugins.append(plugin)

    def load_data(self):
        self.root = tk.Tk()
        self.root.withdraw()
        start_path = path.abspath(path.join(getcwd(), "..", "Replays"))
        
        if not path.exists(start_path):
            start_path = path.abspath(path.join(getcwd(), "Replays"))

        turtle_replay_file = filedialog.askopenfilename(
            title="Select Relay file",
            initialdir=start_path,
            filetypes=[("Replay Files", "*.turtleReplay")]
        )

        self.root.destroy()

        if not turtle_replay_file:
            print("No file selected.")
            exit()

        # turtle_replay_file = r"../Replays/robot1_2024-08-25.004151.turtleReplay"
        # turtle_replay_file = "../Replays/Example.turtleReplay"
        # turtle_replay_file = "Example_Pretty.turtleReplay"

        with zipfile.ZipFile(turtle_replay_file, 'r') as zip_ref:
            zip_ref.extractall("usable_replay")

        file_name = listdir(r"usable_replay/")[0]
        self.title = file_name

        with open(r"usable_replay/" + file_name, 'r', errors="ignore") as curFile:
            file_content = curFile.read()
            json_arrays = file_content.strip().split("\n")
            for json_array in json_arrays:
                self.replay_data.append(json.loads(json_array))

        remove(r"usable_replay/" + file_name)
        rmdir(r"usable_replay/")

        if self.beautify:
            with open("Pretty_JSON.json", 'w') as file:
                file.write(json.dumps(self.replay_data, indent=2))

        self.extract_data()

    def extract_data(self):
        # Verify version of replay is correct
        if "replayVersion" not in self.replay_data[0][0] or self.replay_data[0][0]['replayVersion'] < LATEST_SCHEMA['replayVersion']:
            current_version = self.replay_data[0][0]['replayVersion'] if 'replayVersion' in self.replay_data[0][0] else 0
            latest_version = LATEST_SCHEMA['replayVersion']
            save = messagebox.askyesno(
                    title="Overwrite Existing file?",
                    message=(
                        f"{path.basename(file_name)} is outdated.\n\n"
                        f"Version: {current_version} → {latest_version}\n\n"
                        "Do you want to save the updated file?"
                    )
                )
            self.replay_data = load_fix_and_save(turtle_replay_file, zip_output=turtle_replay_file, output_dir=path.basename(turtle_replay_file), verbose=True, save=save)

        errors = {}
        for line in self.replay_data:
            for entry in line:
                self.total_frames += 1

                for key in self.replay_schema:
                    try:
                        # --- custom extractor ---
                        if key in self.extract_map:
                            self.extract_map[key](entry)
                            continue

                        # --- default extractor ---
                        self.data[key].append(entry[key])

                    except Exception as e:
                        if key not in errors:
                            errors[key] = e
        for key, value in errors.items():
            print(f"Replay extract failed for {key}: {value}")

        # final metadata
        self.neighbor_info = self.data.neighbor_poses[-1]

    def setup(self):

        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=self.size)  # (x, y) x inches wide and y inches tall
        self.fig.subplots_adjust(left=0.35)           # leave 35% of area on left
        self.ax.set_aspect('equal')

        # Set axis limits (adjust based on your data range)
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_title(self.title)

        # Main Robot Marker
        self.robot_marker_circle = patches.Circle((0, 0), radius=self.robot_radius, color='mistyrose', ec='blue', zorder=10)
        self.ax.add_patch(self.robot_marker_circle)

        arrow_length = 1.2 * self.robot_radius
        arrow_dx = arrow_length * np.cos(0)
        arrow_dy = arrow_length * np.sin(0)
        self.robot_marker_arrow = patches.FancyArrowPatch((0, 0), (arrow_dy, arrow_dx), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='blue',
                                                linewidth=2, zorder=12)
        self.ax.add_patch(self.robot_marker_arrow)

        # Trail
        self.trail, = self.ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)
        self.trail_coords = []

        # Neighbor Robot Markers - Use last item to guarentee all are present
        self.neighbor_marker = {}
        self.neighbor_arrow = {}
        for name, pose in self.neighbor_info.items():
            self.neighbor_marker[name] = patches.Circle((self._offgrid[1], self._offgrid[0]), radius=self.robot_radius, color='peachpuff' if self.neighbor_info[name]['in_neighborhood'] else 'lightgray', ec='darkorange' if self.neighbor_info[name]['in_neighborhood'] else 'gray', zorder=8)
            self.ax.add_patch(self.neighbor_marker[name])

            self.neighbor_arrow[name] = patches.FancyArrowPatch((self._offgrid[1], self._offgrid[0]), (self._offgrid[1]+arrow_dy, self._offgrid[0]+arrow_dx), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='darkorange' if pose['in_neighborhood'] else 'gray',
                                                linewidth=2, zorder=9)
            self.neighbor_marker[name].set_facecolor('peachpuff' if pose['in_neighborhood'] else 'lightgray')
            self.neighbor_marker[name].set_edgecolor('darkorange' if pose['in_neighborhood'] else 'gray')
            self.ax.add_patch(self.neighbor_arrow[name])



        # Saving goal destination
        self.goal_marker_x = self.ax.plot(self._offgrid[1], self._offgrid[0], marker='x', color='#FF10F0', markersize=15, zorder=12)[0]
        self.goal_marker_anlge = patches.FancyArrowPatch((self._offgrid[1], self._offgrid[0]), (self._offgrid[1]+arrow_dy, self._offgrid[0]+arrow_dx), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='#FF10F0',
                                                linewidth=2, zorder=13)
        self.ax.add_patch(self.goal_marker_anlge)
        self.goal_radius = patches.Circle((self._offgrid[1], self._offgrid[0]), radius=0.015, color="lightblue", zorder=12, alpha=0.5)
        self.ax.add_patch(self.goal_radius)

        self.goal_attempt_marker_x = self.ax.plot(self._offgrid[1], self._offgrid[0], marker='x', color='#ff1010', markersize=15, zorder=11)[0]

        # Status Indicator
        aspect = self.fig.get_figwidth() / self.fig.get_figheight()
        radius = 0.015

        line_num = 0

        # Zero Line
        self.status_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Robot Status: ', fontsize=self.fontsize, ha='left', va='top')
        self.status_text = self.fig.text(self.start_line_x + 0.11, self.start_line_y - self.delta_line_y*line_num, 'No Status', fontsize=self.fontsize, ha='left', va='top')

        line_num = line_num + 1

        # First line
        self.ready_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.ready_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Robot Not Ready', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.ready_circle)

        line_num = line_num + 1

        # Second line
        self.pos_started_circle = patches.Ellipse((self.start_line_x + self.x_indent, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.pos_started_text = self.fig.text(self.start_line_x + self.x_indent + radius, self.start_line_y - self.delta_line_y*line_num, 'Position Not Obtained', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.pos_started_circle)

        line_num = line_num + 1

        # Third line
        self.neighbors_started_circle = patches.Ellipse((self.start_line_x + self.x_indent, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.neighbors_started_text = self.fig.text(self.start_line_x + self.x_indent + radius, self.start_line_y - self.delta_line_y*line_num, 'Neighbor Position Not Obtained', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.neighbors_started_circle)

        line_num = line_num + 1

        # Forth line
        self.lidar_started_circle = patches.Ellipse((self.start_line_x + self.x_indent, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.lidar_started_text = self.fig.text(self.start_line_x + self.x_indent + radius, self.start_line_y - self.delta_line_y*line_num, 'Lidar Not Obtained', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.lidar_started_circle)

        line_num = line_num + 1

        # Fifth Line
        self.battery_ready_circle = patches.Ellipse((self.start_line_x + self.x_indent, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.battery_ready_text = self.fig.text(self.start_line_x + self.x_indent + radius, self.start_line_y - self.delta_line_y*line_num, 'Battery Not Obtained', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.battery_ready_circle)

        line_num = line_num + 1

        # Sixth line
        self.robot_moving_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.robot_moving_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Robot Not Moving', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.robot_moving_circle)

        line_num = line_num + 1

        # Seventh line
        self.movement_restricted_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.movement_restricted_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Movement Not Restricted', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.movement_restricted_circle)

        line_num = line_num + 1

        # Eighth line
        self.path_obstructed_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.path_obstructed_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Path Not Obstructed', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.path_obstructed_circle)

        line_num = line_num + 1

        # Ninth line
        self.laser_obstructed_circle = patches.Ellipse((self.start_line_x + self.x_indent, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.laser_obstructed_text = self.fig.text(self.start_line_x + self.x_indent + radius, self.start_line_y - self.delta_line_y*line_num, 'Laser Not Obstructed', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.laser_obstructed_circle)

        line_num = line_num + 1

        # Tenth line
        self.neighbor_obstructed_circle = patches.Ellipse((self.start_line_x + self.x_indent, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.neighbor_obstructed_text = self.fig.text(self.start_line_x + self.x_indent + radius, self.start_line_y - self.delta_line_y*line_num, 'Neighbor Not Obstructed', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.neighbor_obstructed_circle)

        line_num = line_num + 1

        # Eleventh line
        self.laser_avoid_error_circle = patches.Ellipse((self.start_line_x + self.x_indent, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.laser_avoid_error_text = self.fig.text(self.start_line_x + self.x_indent + radius, self.start_line_y - self.delta_line_y*line_num, 'No Laser Avoid Error', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.laser_avoid_error_circle)

        line_num = line_num + 1

        # Twelfth line
        self.desired_heading_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.desired_heading_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Desired Heading Not Reached', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.desired_heading_circle)

        line_num = line_num + 1

        # Thirteenth line
        self.destination_reached_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.destination_reached_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Destination Not Reached', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.destination_reached_circle)

        line_num = line_num + 1

        # Fourteenth line
        self.motion_complete_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.motion_complete_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Motion Not Complete', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.motion_complete_circle)

        line_num = line_num + 1

        # Fifteen line
        self.neighbor_complete_circle = patches.Ellipse((self.start_line_x, self.start_line_y - self.delta_line_y*line_num - radius), width=radius, height=radius*aspect, transform=self.fig.transFigure, clip_on=False, color='red')
        self.neighbor_complete_text = self.fig.text(self.start_line_x + radius, self.start_line_y - self.delta_line_y*line_num, 'Neighbors Not Complete', fontsize=self.fontsize, ha='left', va='top')
        self.ax.add_patch(self.neighbor_complete_circle)

        line_num = line_num + 1

        # tolerances
        self.destination_tolerance_text = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, "Destination Tolerance: XXX", fontsize=self.fontsize, ha='left', va='top')
        self.angle_tolerance_text = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y * (line_num+1), "Angle Tolerance: XXX", fontsize=self.fontsize, ha='left', va='top')

        line_num = line_num + 2

        # completion
        self.complete_circle = patches.Ellipse((0.50,0.95), width=radius*2, height=2*radius*aspect, transform=self.fig.transFigure, color='red')
        self.complete_text = self.fig.text(0.50 + radius * 2, 0.96 + radius / 2, "Simulation Running", fontsize=18, ha='left', va='top')
        self.fig.patches.append(self.complete_circle)

        # Adding Widgets
        self.ax_button = plt.axes([0.885, 0.9, 0.08, 0.065])   # Left, bottom, width, height
        self.restart_button = Button(self.ax_button, 'Restart')
        self.ax_pause = plt.axes([0.8, 0.9, 0.08, 0.065])
        self.pause_button = Button(self.ax_pause, 'Pause')

        self.ax_slider = plt.axes([0.35, self.start_line_y - self.delta_line_y * 18.50, 0.53, 0.03])
        self.slider = Slider(self.ax_slider, '', 0, self.total_frames-1, valinit=0, valstep=1)
        self.prev_slider = 0

        # Add LED Ring
        self.led_colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (0, 1, 1)]
        center = (0.37, 0.94)
        self.LED_Label = self.fig.text(0.26, 0.95, "LED Ring:", fontsize=12, ha='left', va='top')

        self.led_ring_patches = []
        angle_step = 2 * np.pi / 5
        led_ring_radius = 0.015
        led_radius = 0.015
        for i in range(5):
            angle = i * angle_step + 23 * np.pi/32
            led_x = center[0] + led_ring_radius * np.cos(angle)
            led_y = center[1] + led_ring_radius * np.sin(angle) * aspect
            led = patches.Ellipse((led_x, led_y), width=led_radius, height=led_radius*aspect, color=self.led_colors[i], transform=self.fig.transFigure)
            self.fig.patches.append(led)
            self.led_ring_patches.append(led)

        self.slider.on_changed(self.slider_changed)
        self.restart_button.on_clicked(self.restart)
        self.pause_button.on_clicked(self.toggle_pause)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        ## Can invert axis if wish
        # self.ax.invert_yaxis()
        self.ax.invert_xaxis()

    def init_graph(self):
        self.robot_marker_circle.xy = (0, 0)    # note they are backwards
        self.robot_marker_circle.set_color('red')

        self.robot_marker_arrow.remove()
        arrow_length = 1.2 * self.robot_radius
        arrow_dx = arrow_length * np.cos(0)
        arrow_dy = arrow_length * np.sin(0)
        self.robot_marker_arrow = patches.FancyArrowPatch((0, 0), (arrow_dy, arrow_dx), 
                                                    arrowstyle='->',
                                                    mutation_scale=20, color='mistyrose',
                                                    linewidth=2, zorder=11)
        self.ax.add_patch(self.robot_marker_arrow)

        # update neighbor position
        for name, pose in self.neighbor_info.items():
            self.neighbor_marker[name].set_center(self._offgrid)
            self.neighbor_arrow[name].remove()
            self.neighbor_arrow[name] = patches.FancyArrowPatch((self._offgrid[1], self._offgrid[0]), (self._offgrid[1]+arrow_dy, self._offgrid[0]+arrow_dx), 
                                            arrowstyle='->',
                                            mutation_scale=20, color='darkorange' if pose['in_neighborhood'] else 'gray',
                                            linewidth=2, zorder=9)
            self.ax.add_patch(self.neighbor_arrow[name])

        # updating goal location 
        if self.goal_marker_x not in self.ax.lines:
            self.goal_marker_x = self.ax.plot(self._offgrid[1], self._offgrid[0], marker='x', color='#FF10F0', markersize=15, zorder=12)[0]
        if self.goal_attempt_marker_x  not in self.ax.lines:
            self.goal_attempt_marker_x = self.ax.plot(self._offgrid[1], self._offgrid[0], marker='x', color='#ff1010', markersize=15, zorder=11)[0]
        if self.trail not in self.ax.lines:
            self.trail, = self.ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)

        self.goal_marker_x.set_data([self._offgrid[1]], [self._offgrid[0]])
        self.goal_attempt_marker_x.set_data([self._offgrid[1]], [self._offgrid[0]])
        self.goal_marker_anlge.remove()
        self.goal_marker_anlge = patches.FancyArrowPatch((self._offgrid[1], self._offgrid[0]), (self._offgrid[1]+arrow_dy, self._offgrid[0]+arrow_dx), 
                                            arrowstyle='->',
                                            mutation_scale=20, color='#FF10F0',
                                            linewidth=2, zorder=12)
        self.ax.add_patch(self.goal_marker_anlge)


        # updating status variables
        self.status_text.set_text('No Status')
        self.ready_text.set_text('Robot Not Ready')
        self.ready_text.set_color('red')
        self.ready_circle.set_color('red')
        self.pos_started_text.set_text('Position Not Obtrained')
        self.pos_started_text.set_color('red')
        self.pos_started_circle.set_color('red')
        self.neighbors_started_text.set_text('Neighbor Position Not Obtrained')
        self.neighbors_started_text.set_color('red')
        self.neighbors_started_circle.set_color('red')
        self.lidar_started_text.set_text('Lidar Not Obtrained')
        self.lidar_started_text.set_color('red')
        self.lidar_started_circle.set_color('red')
        self.battery_ready_text.set_text('Battery Not Obtrained')
        self.battery_ready_text.set_color('red')
        self.battery_ready_circle.set_color('red')
        self.robot_moving_text.set_text('Robot Not Moving')
        self.robot_moving_text.set_color('red')
        self.robot_moving_circle.set_color('red')
        self.movement_restricted_text.set_text('Movement Not Restricted')
        self.movement_restricted_text.set_color('red')
        self.movement_restricted_circle.set_color('red')
        self.path_obstructed_text.set_text('Path Not Obstructed')
        self.path_obstructed_text.set_color('red')
        self.path_obstructed_circle.set_color('red')
        self.laser_obstructed_text.set_text('Laser Not Obstructed')
        self.laser_obstructed_text.set_color('red')
        self.laser_obstructed_circle.set_color('red')
        self.neighbor_obstructed_text.set_text('Neighbor Not Obstructed')
        self.neighbor_obstructed_text.set_color('red')
        self.neighbor_obstructed_circle.set_color('red')
        self.laser_avoid_error_text.set_text('No Laser Avoid Error')
        self.laser_avoid_error_text.set_color('red')
        self.laser_avoid_error_circle.set_color('red')
        self.desired_heading_text.set_text('Desired Heading Not Reached')
        self.desired_heading_text.set_color('red')
        self.desired_heading_circle.set_color('red')
        self.destination_reached_text.set_text('Destination Not Reached')
        self.destination_reached_text.set_color('red')
        self.destination_reached_circle.set_color('red')
        self.motion_complete_text.set_text('Motion Not Completed')
        self.motion_complete_text.set_color('red')
        self.motion_complete_circle.set_color('red')
        self.neighbor_complete_text.set_text('Neighbors Not Completed')
        self.neighbor_complete_text.set_color('red')
        self.neighbor_complete_circle.set_color('red')

        # update tolerances
        self.destination_tolerance_text.set_text(f"Destination Tolerance: XXX")
        self.angle_tolerance_text.set_text(f"Angle Tolerance: XXX")
        self.goal_radius.set_center(self._offgrid)
        self.goal_radius.radius = 1


        self.trail_coords = []
        self.set_slider(0)
        return self.robot_marker_circle

    def update(self, frame):
        frame = self.slider.val

        # update my position
        x,y = self.data.x_vals[frame], self.data.y_vals[frame]
        yaw = self.data.yaws[frame] + np.pi

        self.robot_marker_circle.set_center((y, x))    # note they are backwards
        self.robot_marker_circle.set_color('lightgreen' if self.data.robot_ready[frame] else 'mistyrose')

        self.robot_marker_arrow.remove()
        arrow_length = 1.2 * self.robot_radius
        arrow_dx = arrow_length * np.cos(yaw)
        arrow_dy = arrow_length * np.sin(yaw)
        self.robot_marker_arrow = patches.FancyArrowPatch((y, x), (y+arrow_dy, x+arrow_dx), 
                                                    arrowstyle='->',
                                                    mutation_scale=20, color='blue',
                                                    linewidth=2, zorder=11)
        self.ax.add_patch(self.robot_marker_arrow)

        # update neighbor position
        for name, pose in self.data.neighbor_poses[frame].items():
            self.neighbor_marker[name].set_center((pose['y'], pose['x']))

            self.neighbor_arrow[name].remove()
            neighbor_orientation = pose['yaw'] + np.pi
            arrow_dx = arrow_length * np.cos(neighbor_orientation)
            arrow_dy = arrow_length * np.sin(neighbor_orientation)
            self.neighbor_arrow[name] = patches.FancyArrowPatch((pose['y'], pose['x']), (pose['y']+arrow_dy, pose['x']+arrow_dx), 
                                                        arrowstyle='->',
                                                        mutation_scale=20, color='darkorange' if pose['in_neighborhood'] else 'gray',
                                                        linewidth=2, zorder=9)
            self.neighbor_marker[name].set_facecolor('peachpuff' if pose['in_neighborhood'] else 'lightgray')
            self.neighbor_marker[name].set_edgecolor('darkorange' if pose['in_neighborhood'] else 'gray')
            self.ax.add_patch(self.neighbor_arrow[name])

        # Updating Goal Location 
        if self.data.destination_reached[frame]:
            if self.goal_marker_x in self.ax.lines:
                self.goal_marker_x.remove()
            goal_y, goal_x = self.data.desired_location[frame][1], self.data.desired_location[frame][0]
            goal_ori = self.data.desired_angle[frame] + np.pi
            arrow_dx = arrow_length * np.cos(goal_ori)
            arrow_dy = arrow_length * np.sin(goal_ori)
            self.goal_marker_anlge.remove()
            self.goal_marker_anlge = patches.FancyArrowPatch((goal_y, goal_x), (goal_y+arrow_dy, goal_x+arrow_dx), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='#FF10F0',
                                                linewidth=2, zorder=12)
            self.ax.add_patch(self.goal_marker_anlge)

        else:
            self.goal_marker_anlge.remove()
            self.goal_marker_anlge = patches.FancyArrowPatch((self._offgrid[1], self._offgrid[0]), (0, -100), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='#FF10F0',
                                                linewidth=2, zorder=12)
            self.ax.add_patch(self.goal_marker_anlge)
            if self.goal_marker_x not in self.ax.lines:
                self.goal_marker_x = self.ax.plot(self._offgrid[1], self._offgrid[0], marker='x', color='#FF10F0', markersize=15, zorder=11)[0]
            if type(self.data.desired_location[frame]) != type(None):
                self.goal_marker_x.set_data([self.data.desired_location[frame][1]], [self.data.desired_location[frame][0]])

        if type(self.data.attempted_desired_location[frame]) != type(None):
            self.goal_attempt_marker_x.set_data([self.data.attempted_desired_location[frame][1]], [self.data.attempted_desired_location[frame][0]])

        # Updating Status Variables
        self.status_text.set_text(self.data.robot_status[frame])
        self.ready_text.set_text('Robot Ready' if self.data.robot_ready[frame] else 'Robot Not Ready')
        self.ready_text.set_color('green' if self.data.robot_ready[frame] else 'red')
        self.ready_circle.set_color('green' if self.data.robot_ready[frame] else 'red')
        self.pos_started_text.set_text('Position Obtained' if self.data.position_started[frame] else 'Position Not Obtrained')
        self.pos_started_text.set_color('green' if self.data.position_started[frame] else 'red')
        self.pos_started_circle.set_color('green' if self.data.position_started[frame] else 'red')
        self.neighbors_started_text.set_text('Neighbor Position Obtained' if self.data.neighbors_started[frame] else 'Neighbor Position Not Obtrained')
        self.neighbors_started_text.set_color('green' if self.data.neighbors_started[frame] else 'red')
        self.neighbors_started_circle.set_color('green' if self.data.neighbors_started[frame] else 'red')
        self.lidar_started_text.set_text('Lidar Obtained' if self.data.lidar_started[frame] else 'Lidar Not Obtrained')
        self.lidar_started_text.set_color('green' if self.data.lidar_started[frame] else 'red')
        self.lidar_started_circle.set_color('green' if self.data.lidar_started[frame] else 'red')
        self.battery_ready_text.set_text(f'Battery Obtained {f"- {self.data.battery_dict[frame]['percentage']*100}%" if self.data.battery_dict[frame]['percentage'] else ""}' if not self.data.wait_for_battery[frame] or self.data.battery_received[frame] else ' Battery Not Obtained')
        self.battery_ready_text.set_color('green' if not self.data.wait_for_battery[frame] or self.data.battery_received[frame] else 'red')
        self.battery_ready_circle.set_color('green' if not self.data.wait_for_battery[frame] or self.data.battery_received[frame] else 'red')
        self.robot_moving_text.set_text('Robot Moving' if self.data.robot_moving[frame] else 'Robot Not Moving')
        self.robot_moving_text.set_color('green' if self.data.robot_moving[frame] else 'red')
        self.robot_moving_circle.set_color('green' if self.data.robot_moving[frame] else 'red')
        self.movement_restricted_text.set_text('Movement Restricted' if self.data.movement_restricted[frame] else 'Movement Not Restricted')
        self.movement_restricted_text.set_color('green' if self.data.movement_restricted[frame] else 'red')
        self.movement_restricted_circle.set_color('green' if self.data.movement_restricted[frame] else 'red')
        self.path_obstructed_text.set_text('Path Obstructed' if self.data.path_obstructed[frame] else 'Path Not Obstructed')
        self.path_obstructed_text.set_color('green' if self.data.path_obstructed[frame] else 'red')
        self.path_obstructed_circle.set_color('green' if self.data.path_obstructed[frame] else 'red')
        self.laser_obstructed_text.set_text('Laser Obstructed' if self.data.path_obstructed_laser[frame] else 'Laser Not Obstructed')
        self.laser_obstructed_text.set_color('green' if self.data.path_obstructed_laser[frame] else 'red')
        self.laser_obstructed_circle.set_color('green' if self.data.path_obstructed_laser[frame] else 'red')
        self.neighbor_obstructed_text.set_text('Neighbor Obstructed' if self.data.path_obstructed_neighbor[frame] else 'Neighbor Not Obstructed')
        self.neighbor_obstructed_text.set_color('green' if self.data.path_obstructed_neighbor[frame] else 'red')
        self.neighbor_obstructed_circle.set_color('green' if self.data.path_obstructed_neighbor[frame] else 'red')
        self.laser_avoid_error_text.set_text('Laser Avoid Error' if self.data.laser_avoid_error[frame] else 'No Laser Avoid Error')
        self.laser_avoid_error_text.set_color('green' if self.data.laser_avoid_error[frame] else 'red')
        self.laser_avoid_error_circle.set_color('green' if self.data.laser_avoid_error[frame] else 'red')
        self.desired_heading_text.set_text('Desired Heading Reached' if self.data.desired_heading[frame] else 'Desired Heading Not Reached')
        self.desired_heading_text.set_color('green' if self.data.desired_heading[frame] else 'red')
        self.desired_heading_circle.set_color('green' if self.data.desired_heading[frame] else 'red')
        self.destination_reached_text.set_text('Destination Reached' if self.data.destination_reached[frame] else 'Destination Not Reached')
        self.destination_reached_text.set_color('green' if self.data.destination_reached[frame] else 'red')
        self.destination_reached_circle.set_color('green' if self.data.destination_reached[frame] else 'red')
        self.motion_complete_text.set_text('Motion Completed' if self.data.motion_complete[frame] else 'Motion Not Completed')
        self.motion_complete_text.set_color('green' if self.data.motion_complete[frame] else 'red')
        self.motion_complete_circle.set_color('green' if self.data.motion_complete[frame] else 'red')
        self.neighbor_complete_text.set_text('Neighbors Completed' if self.data.neighbors_complete[frame] else 'Neighbors Not Completed')
        self.neighbor_complete_text.set_color('green' if self.data.neighbors_complete[frame] else 'red')
        self.neighbor_complete_circle.set_color('green' if self.data.neighbors_complete[frame] else 'red')

        # update tolerances
        self.destination_tolerance_text.set_text(f"Destination Tolerance: {self.data.destination_tolerance[frame]}")
        self.angle_tolerance_text.set_text(f"Angle Tolerance: {self.data.angle_tolerance[frame]}")
        if type(self.data.desired_location[frame]) != type(None):
            new_goal_radius = self.data.destination_tolerance[frame]
            self.goal_radius.set_center((self.data.desired_location[frame][1], self.data.desired_location[frame][0]))
            self.goal_radius.radius = new_goal_radius

        # update LED Ring
        if type(self.data.led_light_state[frame]) != type(None):
            self.led_colors = []
            for led in self.data.led_light_state[frame]['leds']:
                self.led_colors.append((led['red']/255, led['green']/255, led['blue']/255))
            self.update_led_ring(self.led_colors)

        self.trail_coords.append((y, x))
        if len(self.trail_coords) > self.trail_length:
            self.trail_coords.pop(0)
        
        if self.trail not in self.ax.lines:
            self.trail, = self.ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)
        self.trail.set_data(*zip(*self.trail_coords))

        if frame ==  self.total_frames - 1:
            self.on_animation_complete()
        else:
            self.set_slider(frame+1)

        # Allow for other things...
        for plugin in self.plugins:
            plugin.update(self, frame)

        return self.robot_marker_circle

    def set_slider(self, val):
        self.prev_slider = self.slider.val

        self.slider.eventson = False
        self.slider.set_val(val)
        self.slider.eventson = True

    def slider_changed(self, val):
        if val < self.prev_slider:
            self.ani.event_source.stop()
            self.ani.frame_seq = self.ani.new_frame_seq()
            if not self.paused:
                self.ani.event_source.start()

    def on_animation_complete(self):
        self.complete_circle.set_color('green')
        self.complete_text.set_text("Simulation Complete")
        self.ani.event_source.stop()
        self.fig.canvas.draw_idle()

    def start_animation(self):
        self.ani = animation.FuncAnimation(
            self.fig, 
            self.update, 
            init_func=self.init_graph,
            frames=self.total_frames, 
            interval=1000/self.frame_rate, 
            blit=False, repeat=False)
        self.fig.canvas.draw_idle()
    
    def restart(self, event):
        self.complete_circle.set_color('red')
        self.complete_text.set_text('Motion Not Complete')


        self.goal_marker_anlge.remove()
        self.goal_marker_anlge = patches.FancyArrowPatch((self._offgrid[1], self._offgrid[0]), (0, self._offgrid[0]), 
                                            arrowstyle='->',
                                            mutation_scale=20, color='#FF10F0',
                                            linewidth=2, zorder=12)
        self.ax.add_patch(self.goal_marker_anlge)
        if self.goal_marker_x in self.ax.lines:
            self.goal_marker_x.remove()
        self.goal_attempt_marker_x.set_data([self._offgrid[1]], [self._offgrid[0]])
        if self.trail in self.ax.lines:
            self.trail.remove()
        self.trail_coords = []

        if type(self.ani.event_source) != type(None):
            self.ani.event_source.stop()
            self.set_slider(0)
            self.ani.frame_seq = self.ani.new_frame_seq()
            self.ani.event_source.start()
        else:
            self.start_animation()

    def toggle_pause(self, event):
        if self.paused:
            self.ani.event_source.start()
            self.pause_button.label.set_text("Pause")
        else:
            self.ani.event_source.stop()
            self.pause_button.label.set_text("Play")

        self.paused = not self.paused
        self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key == ' ':
            self.toggle_pause(None)

    def update_led_ring(self, led_colers=[]):
        """
        Update a 5-LED ring on the given matplotlib axis.

        Parameters:
        - led_colors: list of 5 RGB tuples (R, G, B) values in 0–1
        """
        for i, led in enumerate(self.led_ring_patches):
            led.set_color(self.led_colors[i])

    def _save_mp4(self):
        print("Saving mp4 File...")
        self.ani.save(self.filename, writer='ffmpeg', fps=self.frame_rate)

    def _play(self):
        plt.show()
    
    def run(self):
        if self.save:
            self._save_mp4()
        if self.play:
            self._play()


def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--save", default=False, action="store_true", help="Save MP4")
    parser.add_argument("-p", "--play", default=True, action="store_false", help="Set to not show graph")
    parser.add_argument("-f", "--filename", default="Example", type=str, help="Name of MP4 file without .mp4")
    parser.add_argument("-b", "--beauty", default=False, action="store_true", help="Save Pretty Json")
    script_args = parser.parse_args()

    replayVisual = ReplayVisualizer(script_args.play, script_args.save, script_args.filename, script_args.beauty)
    replayVisual.frame_rate = 10    # 10 frames is "Real Time"
    replayVisual.load_data()
    replayVisual.setup()


    replayVisual.start_animation()
    replayVisual.run()

# pdb.set_trace()
if __name__ == '__main__':
    main()