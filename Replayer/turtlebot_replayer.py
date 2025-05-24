import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, Slider
import matplotlib.animation as animation
import numpy as np
from os import listdir, remove, rmdir
import zipfile
import pdb

turtle_replay_file = r"../Replays/robot1_2024-08-25.004151.turtleReplay"
# turtle_replay_file = "../Replays/Example.turtleReplay"
# turtle_replay_file = "Example_Pretty.turtleReplay"

trail_length = 100
save_mp4 = False
mp4_file_name = "Example.mp4"

with zipfile.ZipFile(turtle_replay_file, 'r') as zip_ref:
    zip_ref.extractall("usable_replay")

file_name = listdir(r"usable_replay/")[0]

data = []

with open(r"usable_replay/" + file_name, 'r', errors="ignore") as curFile:
    file_content = curFile.read()
    json_arrays = file_content.strip().split("\n")
    for json_array in json_arrays:
        data.append(json.loads(json_array))

remove(r"usable_replay/" + file_name)
rmdir(r"usable_replay/")

# Extract informaiton
x_vals, y_vals, yaws, neighbor_poses = [], [], [], []
robot_ready, position_started, neighbors_started, lidar_started, robot_moving = [], [], [], [], []
desired_heading, destination_reached, motion_complete, neighbors_complete = [], [], [], []
movement_restricted, path_obstructed, path_obstructed_laser, path_obstructed_neighbor, laser_avoid_error = [], [], [], [], []
destination_tolerance, angle_tolerance, desired_location, desired_angle = [], [], [], []

total_frames = 0

for line in data:
    for entry in line:
        total_frames += 1

        # Saving status Booleans
        # ## TODO: TEMPERARY
        # entry['neighbors_started'] = entry['neighors_started']


        robot_ready.append(entry['robot_ready'])
        position_started.append(entry['position_started'])
        neighbors_started.append(entry['neighbors_started'])
        lidar_started.append(entry['lidar_started'])
        robot_moving.append(entry['robot_moving'])
        desired_heading.append(entry['desired_heading'])
        destination_reached.append(entry['destination_reached'])
        motion_complete.append(entry['motion_complete'])
        neighbors_complete.append(entry['neighbors_complete'])
        movement_restricted.append(entry['movement_restricted'])
        path_obstructed.append(entry['path_obstructed'])
        path_obstructed_laser.append(entry['path_obstructed_laser'])
        path_obstructed_neighbor.append(entry['path_obstructed_neighbor'])
        laser_avoid_error.append(entry['laser_avoid_error'])

        # Goal Information
        destination_tolerance.append(entry['destination_tolerance'])
        angle_tolerance.append(entry['angle_tolerance'])
        desired_location.append(entry['desired_location'])
        desired_angle.append(entry['desired_angle'])

        # Saving main robot information
        if type(entry['my_pose']) != type(None):
            x_vals.append(entry['my_pose']['pose']['position']['x'])
            y_vals.append(entry['my_pose']['pose']['position']['y'])
            ori = entry['my_pose']['pose']['orientation']
            qx, qy, qz, qw = ori['x'], ori['y'], ori['z'], ori['w']
            yaw = np.remainder((np.arctan2(2 * (qw * qz + qx * qy),1 - 2 * (qy * qy + qz * qz)) + np.pi) , 2 * np.pi)
            yaws.append(yaw)
        else:
            x_vals.append(0)
            y_vals.append(0)
            yaws.append(0)


        # Saving Neighbors Info
        neighbor_poses.append({})
        for name, pose in entry['neighbor_poses'].items():
            ori = pose['pose']['orientation']
            qx, qy, qz, qw = ori['x'], ori['y'], ori['z'], ori['w']
            neighbor_poses[-1][name] = {
                'x': pose['pose']['position']['x'],
                'y': pose['pose']['position']['y'],
                'yaw': np.remainder((np.arctan2(2 * (qw * qz + qx * qy),1 - 2 * (qy * qy + qz * qz)) + np.pi) , 2 * np.pi),
            }



# Set up the plot
fig, ax = plt.subplots(figsize=(10,6))  # (x, y) x inches wide and y inches tall
fig.subplots_adjust(left=0.35)           # leave 35% of area on left

scat = ax.plot([], [], 'bo')[0]  # robot's position as a blue dot
text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12, verticalalignment='top')

# Set axis limits (adjust based on your data range)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_title("Robot Position Over Time")

robot_radius = 0.4

# Main Robot Marker
robot_marker_circle = patches.Circle((0, 0), radius=robot_radius, color='mistyrose', ec='blue', zorder=10)
ax.add_patch(robot_marker_circle)

arrow_length = 1.2 * robot_radius
arrow_dx = arrow_length * np.cos(0)
arrow_dy = arrow_length * np.sin(0)
robot_marker_arrow = patches.FancyArrowPatch((0, 0), (arrow_dy, arrow_dx), 
                                        arrowstyle='->',
                                        mutation_scale=20, color='blue',
                                        linewidth=2, zorder=11)
ax.add_patch(robot_marker_arrow)

# Trail
trail, = ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)
trail_coords = []

# Neighbor Robot Markers - Use last item to guarentee all are present
neighbor_marker = {}
neighbor_arrow = {}
for name, pose in data[-1][-1]["neighbor_poses"].items():
    neighbor_marker[name] = patches.Circle((-100, -100), radius=robot_radius, color='peachpuff', ec='darkorange', zorder=8)
    ax.add_patch(neighbor_marker[name])

    neighbor_arrow[name] = patches.FancyArrowPatch((-100, -100), (-100+arrow_dy, -100+arrow_dx), 
                                        arrowstyle='->',
                                        mutation_scale=20, color='darkorange',
                                        linewidth=2, zorder=9)
    ax.add_patch(neighbor_arrow[name])



# Saving goal destination
goal_marker_x = ax.plot(-100,-100, marker='x', color='#FF10F0', markersize=15, zorder=11)[0]
goal_marker_anlge = patches.FancyArrowPatch((-100, -100), (-100+arrow_dy, -100+arrow_dx), 
                                        arrowstyle='->',
                                        mutation_scale=20, color='#FF10F0',
                                        linewidth=2, zorder=12)
ax.add_patch(goal_marker_anlge)
goal_radius = patches.Circle((-100, -100), radius=0.015, color="lightblue", zorder=8)
ax.add_patch(goal_radius)

# Status Indicator
aspect = fig.get_figwidth() / fig.get_figheight()
radius = 0.015

start_line_x = 0.02
start_line_y = 0.95
delta_line_x = 0.1
delta_line_y = 0.060
x_indent = 0.015

# First line
status_circle = patches.Ellipse((start_line_x, start_line_y - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
status_text = fig.text(start_line_x + radius, start_line_y, 'Robot Not Ready', fontsize=12, ha='left', va='top')
ax.add_patch(status_circle)

# Second line
pos_started_circle = patches.Ellipse((start_line_x + x_indent, start_line_y - delta_line_y - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
pos_started_text = fig.text(start_line_x + x_indent + radius, start_line_y - delta_line_y, 'Position Not Obtained', fontsize=12, ha='left', va='top')
ax.add_patch(pos_started_circle)

# Third line
neighbors_started_circle = patches.Ellipse((start_line_x + x_indent, start_line_y - delta_line_y * 2 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
neighbors_started_text = fig.text(start_line_x + x_indent + radius, start_line_y - delta_line_y * 2, 'Neighbor Position Not Obtained', fontsize=12, ha='left', va='top')
ax.add_patch(neighbors_started_circle)

# Forth line
lidar_started_circle = patches.Ellipse((start_line_x + x_indent, start_line_y - delta_line_y * 3 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
lidar_started_text = fig.text(start_line_x + x_indent + radius, start_line_y - delta_line_y * 3, 'Lidar Not Obtained', fontsize=12, ha='left', va='top')
ax.add_patch(lidar_started_circle)

# Fifth line
robot_moving_circle = patches.Ellipse((start_line_x, start_line_y - delta_line_y * 4 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
robot_moving_text = fig.text(start_line_x + radius, start_line_y - delta_line_y * 4, 'Robot Not Moving', fontsize=12, ha='left', va='top')
ax.add_patch(robot_moving_circle)

# Sixth line
movement_restricted_circle = patches.Ellipse((start_line_x, start_line_y - delta_line_y * 5 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
movement_restricted_text = fig.text(start_line_x + radius, start_line_y - delta_line_y * 5, 'Movement Not Restricted', fontsize=12, ha='left', va='top')
ax.add_patch(movement_restricted_circle)

# Seventh line
path_obstructed_circle = patches.Ellipse((start_line_x, start_line_y - delta_line_y * 6 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
path_obstructed_text = fig.text(start_line_x + radius, start_line_y - delta_line_y * 6, 'Path Not Obstructed', fontsize=12, ha='left', va='top')
ax.add_patch(path_obstructed_circle)

# Eighth line
laser_obstructed_circle = patches.Ellipse((start_line_x + x_indent, start_line_y - delta_line_y * 7 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
laser_obstructed_text = fig.text(start_line_x + x_indent + radius, start_line_y - delta_line_y * 7, 'Laser Not Obstructed', fontsize=12, ha='left', va='top')
ax.add_patch(laser_obstructed_circle)

# Ninth line
neighbor_obstructed_circle = patches.Ellipse((start_line_x + x_indent, start_line_y - delta_line_y * 8 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
neighbor_obstructed_text = fig.text(start_line_x + x_indent + radius, start_line_y - delta_line_y * 8, 'Neighbor Not Obstructed', fontsize=12, ha='left', va='top')
ax.add_patch(neighbor_obstructed_circle)

# Tenth line
laser_avoid_error_circle = patches.Ellipse((start_line_x + x_indent, start_line_y - delta_line_y * 9 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
laser_avoid_error_text = fig.text(start_line_x + x_indent + radius, start_line_y - delta_line_y * 9, 'No Laser Avoid Error', fontsize=12, ha='left', va='top')
ax.add_patch(laser_avoid_error_circle)

# Eleventh line
desired_heading_circle = patches.Ellipse((start_line_x, start_line_y - delta_line_y * 10 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
desired_heading_text = fig.text(start_line_x + radius, start_line_y - delta_line_y * 10, 'Desired Heading Not Reached', fontsize=12, ha='left', va='top')
ax.add_patch(desired_heading_circle)

# Twelfth line
destination_reached_circle = patches.Ellipse((start_line_x, start_line_y - delta_line_y * 11 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
destination_reached_text = fig.text(start_line_x + radius, start_line_y - delta_line_y * 11, 'Destination Not Reached', fontsize=12, ha='left', va='top')
ax.add_patch(destination_reached_circle)

# Thirteenth line
motion_complete_circle = patches.Ellipse((start_line_x, start_line_y - delta_line_y * 12 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
motion_complete_text = fig.text(start_line_x + radius, start_line_y - delta_line_y * 12, 'Motion Not Complete', fontsize=12, ha='left', va='top')
ax.add_patch(motion_complete_circle)

# Fourteenth line
neighbor_complete_circle = patches.Ellipse((start_line_x, start_line_y - delta_line_y * 13 - radius), width=radius, height=radius*aspect, transform=fig.transFigure, clip_on=False, color='red')
neighbor_complete_text = fig.text(start_line_x + radius, start_line_y - delta_line_y * 13, 'Neighbors Not Complete', fontsize=12, ha='left', va='top')
ax.add_patch(neighbor_complete_circle)

# tolerances
destination_tolerance_text = fig.text(start_line_x, start_line_y - delta_line_y * 14, "Destination Tolerance: XXX", fontsize=12, ha='left', va='top')
angle_tolerance_text = fig.text(start_line_x, start_line_y - delta_line_y * 15, "Angle Tolerance: XXX", fontsize=12, ha='left', va='top')

# completion
complete_circle = patches.Ellipse((0.50,0.95), width=radius*2, height=2*radius*aspect, transform=fig.transFigure, color='red')
complete_text = fig.text(0.50 + radius * 2, 0.96 + radius / 2, "Simulation Running", fontsize=18, ha='left', va='top')
fig.patches.append(complete_circle)

# Adding Widgets
ax_button = plt.axes([0.85, 0.9, 0.1, 0.065])   # Left, bottom, width, height
restart_button = Button(ax_button, 'Restart')

ax_slider = plt.axes([0.35, start_line_y - delta_line_y * 15.5, 0.53, 0.03])
slider = Slider(ax_slider, '', 0, total_frames-1, valinit=0, valstep=1)
prev_slider = 0

'''
I want to show the frame number so I know where it is at in the file
'''
def init():
    global goal_marker_x
    global trail
    global trail_coords
    global robot_radius
    global robot_marker_arrow
    global goal_marker_anlge

    robot_marker_circle.xy = (0, 0)    # note they are backwards
    robot_marker_circle.set_color('red')

    robot_marker_arrow.remove()
    arrow_length = 1.2 * robot_radius
    arrow_dx = arrow_length * np.cos(0)
    arrow_dy = arrow_length * np.sin(0)
    robot_marker_arrow = patches.FancyArrowPatch((0, 0), (arrow_dy, arrow_dx), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='mistyrose',
                                                linewidth=2, zorder=11)
    ax.add_patch(robot_marker_arrow)

    # update neighbor position
    for name, pose in neighbor_poses[-1].items():
        neighbor_marker[name].set_center((-100, -100))
        neighbor_arrow[name].remove()
        neighbor_arrow[name] = patches.FancyArrowPatch((-100, -100), (-100+arrow_dy, -100+arrow_dx), 
                                        arrowstyle='->',
                                        mutation_scale=20, color='darkorange',
                                        linewidth=2, zorder=9)
        ax.add_patch(neighbor_arrow[name])

    # updating goal location 
    if goal_marker_x not in ax.lines:
        goal_marker_x = ax.plot(-100,-100, marker='x', color='#FF10F0', markersize=15, zorder=11)[0]
    if trail not in ax.lines:
        trail, = ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)

    goal_marker_x.set_data([-100], [-100])
    goal_marker_anlge.remove()
    goal_marker_anlge = patches.FancyArrowPatch((-100, -100), (-100+arrow_dy, -100+arrow_dx), 
                                        arrowstyle='->',
                                        mutation_scale=20, color='#FF10F0',
                                        linewidth=2, zorder=12)
    ax.add_patch(goal_marker_anlge)


    # updating status variables
    status_text.set_text('Robot Not Ready')
    status_text.set_color('red')
    status_circle.set_color('red')
    pos_started_text.set_text('Position Not Obtrained')
    pos_started_text.set_color('red')
    pos_started_circle.set_color('red')
    neighbors_started_text.set_text('Neighbor Position Not Obtrained')
    neighbors_started_text.set_color('red')
    neighbors_started_circle.set_color('red')
    lidar_started_text.set_text('Lidar Not Obtrained')
    lidar_started_text.set_color('red')
    lidar_started_circle.set_color('red')
    robot_moving_text.set_text('Robot Not Moving')
    robot_moving_text.set_color('red')
    robot_moving_circle.set_color('red')
    movement_restricted_text.set_text('Movement Not Restricted')
    movement_restricted_text.set_color('red')
    movement_restricted_circle.set_color('red')
    path_obstructed_text.set_text('Path Not Obstructed')
    path_obstructed_text.set_color('red')
    path_obstructed_circle.set_color('red')
    laser_obstructed_text.set_text('Laser Not Obstructed')
    laser_obstructed_text.set_color('red')
    laser_obstructed_circle.set_color('red')
    neighbor_obstructed_text.set_text('Neighbor Not Obstructed')
    neighbor_obstructed_text.set_color('red')
    neighbor_obstructed_circle.set_color('red')
    laser_avoid_error_text.set_text('No Laser Avoid Error')
    laser_avoid_error_text.set_color('red')
    laser_avoid_error_circle.set_color('red')
    desired_heading_text.set_text('Desired Heading Not Reached')
    desired_heading_text.set_color('red')
    desired_heading_circle.set_color('red')
    destination_reached_text.set_text('Destination Not Reached')
    destination_reached_text.set_color('red')
    destination_reached_circle.set_color('red')
    motion_complete_text.set_text('Motion Not Completed')
    motion_complete_text.set_color('red')
    motion_complete_circle.set_color('red')
    neighbor_complete_text.set_text('Neighbors Not Completed')
    neighbor_complete_text.set_color('red')
    neighbor_complete_circle.set_color('red')

    # update tolerances
    destination_tolerance_text.set_text(f"Destination Tolerance: XXX")
    angle_tolerance_text.set_text(f"Angle Tolerance: XXX")
    goal_radius.set_center((-100,-100))
    goal_radius.radius = 1


    trail_coords = []
    set_slider(0)
    
    return robot_marker_circle

def update(frame):
    global goal_marker_x
    global aspect
    global trail
    global trail_coords
    global robot_radius
    global robot_marker_arrow
    global goal_marker_anlge

    frame = slider.val

    # update my position
    x,y = x_vals[frame], y_vals[frame]
    yaw = yaws[frame] + np.pi

    robot_marker_circle.set_center((y, x))    # note they are backwards
    robot_marker_circle.set_color('lightgreen' if robot_ready[frame] else 'mistyrose')

    robot_marker_arrow.remove()
    arrow_length = 1.2 * robot_radius
    arrow_dx = arrow_length * np.cos(yaw)
    arrow_dy = arrow_length * np.sin(yaw)
    robot_marker_arrow = patches.FancyArrowPatch((y, x), (y+arrow_dy, x+arrow_dx), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='blue',
                                                linewidth=2, zorder=11)
    ax.add_patch(robot_marker_arrow)

    # update neighbor position
    for name, pose in neighbor_poses[frame].items():
        neighbor_marker[name].set_center((pose['y'], pose['x']))

        neighbor_arrow[name].remove()
        neighbor_orientation = pose['yaw'] + np.pi
        arrow_dx = arrow_length * np.cos(neighbor_orientation)
        arrow_dy = arrow_length * np.sin(neighbor_orientation)
        neighbor_arrow[name] = patches.FancyArrowPatch((pose['y'], pose['x']), (pose['y']+arrow_dy, pose['x']+arrow_dx), 
                                                    arrowstyle='->',
                                                    mutation_scale=20, color='darkorange',
                                                    linewidth=2, zorder=9)
        ax.add_patch(neighbor_arrow[name])

    # updating goal location 
    if destination_reached[frame]:
        if goal_marker_x in ax.lines:
            goal_marker_x.remove()
        goal_y, goal_x = desired_location[frame][1], desired_location[frame][0]
        goal_ori = desired_angle[frame] + np.pi
        arrow_dx = arrow_length * np.cos(goal_ori)
        arrow_dy = arrow_length * np.sin(goal_ori)
        goal_marker_anlge.remove()
        goal_marker_anlge = patches.FancyArrowPatch((goal_y, goal_x), (goal_y+arrow_dy, goal_x+arrow_dx), 
                                            arrowstyle='->',
                                            mutation_scale=20, color='#FF10F0',
                                            linewidth=2, zorder=12)
        ax.add_patch(goal_marker_anlge)

    else:
        goal_marker_anlge.remove()
        goal_marker_anlge = patches.FancyArrowPatch((-100, -100), (0, -100), 
                                            arrowstyle='->',
                                            mutation_scale=20, color='#FF10F0',
                                            linewidth=2, zorder=12)
        ax.add_patch(goal_marker_anlge)
        if goal_marker_x not in ax.lines:
            goal_marker_x = ax.plot(-100,-100, marker='x', color='#FF10F0', markersize=15, zorder=11)[0]
        if type(desired_location[frame]) != type(None):
            goal_marker_x.set_data([desired_location[frame][1]], [desired_location[frame][0]])

    # updating status variables
    status_text.set_text('Robot Ready' if robot_ready[frame] else 'Robot Not Ready')
    status_text.set_color('green' if robot_ready[frame] else 'red')
    status_circle.set_color('green' if robot_ready[frame] else 'red')
    pos_started_text.set_text('Position Obtained' if position_started[frame] else 'Position Not Obtrained')
    pos_started_text.set_color('green' if position_started[frame] else 'red')
    pos_started_circle.set_color('green' if position_started[frame] else 'red')
    neighbors_started_text.set_text('Neighbor Position Obtained' if neighbors_started[frame] else 'Neighbor Position Not Obtrained')
    neighbors_started_text.set_color('green' if neighbors_started[frame] else 'red')
    neighbors_started_circle.set_color('green' if neighbors_started[frame] else 'red')
    lidar_started_text.set_text('Lidar Obtained' if lidar_started[frame] else 'Lidar Not Obtrained')
    lidar_started_text.set_color('green' if lidar_started[frame] else 'red')
    lidar_started_circle.set_color('green' if lidar_started[frame] else 'red')
    robot_moving_text.set_text('Robot Moving' if robot_moving[frame] else 'Robot Not Moving')
    robot_moving_text.set_color('green' if robot_moving[frame] else 'red')
    robot_moving_circle.set_color('green' if robot_moving[frame] else 'red')
    movement_restricted_text.set_text('Movement Restricted' if movement_restricted[frame] else 'Movement Not Restricted')
    movement_restricted_text.set_color('green' if movement_restricted[frame] else 'red')
    movement_restricted_circle.set_color('green' if movement_restricted[frame] else 'red')
    path_obstructed_text.set_text('Path Obstructed' if path_obstructed[frame] else 'Path Not Obstructed')
    path_obstructed_text.set_color('green' if path_obstructed[frame] else 'red')
    path_obstructed_circle.set_color('green' if path_obstructed[frame] else 'red')
    laser_obstructed_text.set_text('Laser Obstructed' if path_obstructed_laser[frame] else 'Laser Not Obstructed')
    laser_obstructed_text.set_color('green' if path_obstructed_laser[frame] else 'red')
    laser_obstructed_circle.set_color('green' if path_obstructed_laser[frame] else 'red')
    neighbor_obstructed_text.set_text('Neighbor Obstructed' if path_obstructed_neighbor[frame] else 'Neighbor Not Obstructed')
    neighbor_obstructed_text.set_color('green' if path_obstructed_neighbor[frame] else 'red')
    neighbor_obstructed_circle.set_color('green' if path_obstructed_neighbor[frame] else 'red')
    laser_avoid_error_text.set_text('Laser Avoid Error' if laser_avoid_error[frame] else 'No Laser Avoid Error')
    laser_avoid_error_text.set_color('green' if laser_avoid_error[frame] else 'red')
    laser_avoid_error_circle.set_color('green' if laser_avoid_error[frame] else 'red')
    desired_heading_text.set_text('Desired Heading Reached' if desired_heading[frame] else 'Desired Heading Not Reached')
    desired_heading_text.set_color('green' if desired_heading[frame] else 'red')
    desired_heading_circle.set_color('green' if desired_heading[frame] else 'red')
    destination_reached_text.set_text('Destination Reached' if destination_reached[frame] else 'Destination Not Reached')
    destination_reached_text.set_color('green' if destination_reached[frame] else 'red')
    destination_reached_circle.set_color('green' if destination_reached[frame] else 'red')
    motion_complete_text.set_text('Motion Completed' if motion_complete[frame] else 'Motion Not Completed')
    motion_complete_text.set_color('green' if motion_complete[frame] else 'red')
    motion_complete_circle.set_color('green' if motion_complete[frame] else 'red')
    neighbor_complete_text.set_text('Neighbors Completed' if neighbors_complete[frame] else 'Neighbors Not Completed')
    neighbor_complete_text.set_color('green' if neighbors_complete[frame] else 'red')
    neighbor_complete_circle.set_color('green' if neighbors_complete[frame] else 'red')

    # update tolerances
    destination_tolerance_text.set_text(f"Destination Tolerance: {destination_tolerance[frame]}")
    angle_tolerance_text.set_text(f"Angle Tolerance: {angle_tolerance[frame]}")
    if type(desired_location[frame]) != type(None):
        goal_radius.set_center((desired_location[frame][1], desired_location[frame][0]))
        goal_radius.radius = destination_tolerance[frame]


    trail_coords.append((y, x))
    if len(trail_coords) > trail_length:
        trail_coords.pop(0)
    
    if trail not in ax.lines:
        trail, = ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)
    trail.set_data(*zip(*trail_coords))

    if frame ==  total_frames - 1:
        on_animation_complete()
    else:
        set_slider(frame+1)
    return robot_marker_circle

def set_slider(val):
    global prev_slider
    prev_slider = slider.val

    slider.eventson = False
    slider.set_val(val)
    slider.eventson = True

def slider_changed(val):
    global prev_slider

    if val < prev_slider:
        ani.event_source.stop()
        ani.frame_seq = ani.new_frame_seq()
        ani.event_source.start()
    
        
def on_animation_complete():
    complete_circle.set_color('green')
    complete_text.set_text("Simulation Complete")
    ani.event_source.stop()
    fig.canvas.draw_idle()

def start_animation():
    global ani
    global fig
    global update
    global total_frames
    global frame_rate

    ani = animation.FuncAnimation(
        fig, 
        update, 
        init_func=init,
        frames=total_frames, 
        interval=1000/frame_rate, 
        blit=False, repeat=False)
    fig.canvas.draw_idle()

def restart(event):
    global ani
    global trail
    global trail_coords
    goal_marker_anlge

    complete_circle.set_color('red')
    complete_text.set_text('Motion Not Complete')


    goal_marker_anlge.remove()
    goal_marker_anlge = patches.FancyArrowPatch((-100, -100), (0, -100), 
                                        arrowstyle='->',
                                        mutation_scale=20, color='#FF10F0',
                                        linewidth=2, zorder=12)
    ax.add_patch(goal_marker_anlge)
    if goal_marker_x in ax.lines:
        goal_marker_x.remove()
    if trail in ax.lines:
        trail.remove()
    trail_coords = []

    if type(ani.event_source) != type(None):
        ani.event_source.stop()
        set_slider(0)
        ani.frame_seq = ani.new_frame_seq()
        ani.event_source.start()
    else:
        start_animation()

slider.on_changed(slider_changed)
# interval is time between frames: 100 = 10 frams per second
frame_rate = 10 # 10 frames is "Real time"
start_animation()
# ani = animation.FuncAnimation(fig, update, frames=total_frames, interval=1000/frame_rate, blit=False, repeat=False)
restart_button.on_clicked(restart)

## Can invert axis if wish
#ax.invert_yaxis()
#ax.invert_xaxis()

## Save to MP4 (requires ffmpeg)
if save_mp4:
    print("Saving mp4 File...")
    ani.save(mp4_file_name, writer='ffmpeg', fps=frame_rate)

plt.show()

# pdb.set_trace()
