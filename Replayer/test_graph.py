import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
from os import listdir, remove, rmdir
import zipfile
import pdb

with zipfile.ZipFile("Example.turtleReplay", 'r') as zip_ref:
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
data = data[::3]   # get every nth sample
x_vals, y_vals, yaws, ready_flags = [], [], [], []
for entry in data:
    x_vals.append(entry['my_pose']['pose']['position']['x'])
    y_vals.append(entry['my_pose']['pose']['position']['y'])
    ori = entry['my_pose']['pose']['orientation']

    qx, qy, qz, qw = ori['x'], ori['y'], ori['z'], ori['w']
    yaw = np.remainder((np.arctan2(2 * (qw * qz + qx * qy),1 - 2 * (qy * qy + qz * qz)) + np.pi) , 2 * np.pi)
    yaws.append(yaw)

    ready_flags.append(entry['robot_ready'])


# Set up the plot
fig, ax = plt.subplots()
scat = ax.plot([], [], 'bo')[0]  # robot's position as a blue dot
text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12, verticalalignment='top')

# Set axis limits (adjust based on your data range)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_title("Robot Position Over Time")

# Main Robot Marker
robot_marker = patches.RegularPolygon((0,0), numVertices=3, radius=0.4, orientation=0, color='blue', zorder=10)
ax.add_patch(robot_marker)

# Neighbor Robot Markers - Use last item to guarentee all are present
neighbor_marker = []
for name, pose in data[-1]["neighbor_poses"].items():
    pass


# Trail
trail, = ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)
trail_coords = []

# Status Indicator
status_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
status_circle = patches.Circle((0.02, 0.95), 0.015, transform=ax.transAxes, clip_on=False)
ax.add_patch(status_circle)

# completion
complete_circle = patches.Circle((0.9,0.9), 0.03, transform=fig.transFigure, color='red')
fig.patches.append(complete_circle)

'''
I want to show the frame number so I know where it is at in the file
'''
def update(frame):

    x,y = x_vals[frame], y_vals[frame]
    yaw = yaws[frame]
    ready = ready_flags[frame]

    robot_marker.xy = (y, x)    # note they are backwards
    robot_marker.orientation = yaw + np.pi
    robot_marker.set_color('green' if ready else 'red')

    status_text.set_text('Ready' if ready else 'Not Ready')
    status_text.set_color('green' if ready else 'red')
    status_circle.set_facecolor('green' if ready else 'red')

    trail_coords.append((y, x))
    if len(trail_coords) > 10:
        trail_coords.pop(0)
    trail.set_data(*zip(*trail_coords))

    if frame == len(data) - 1:
        on_animation_complete()
    
    return robot_marker, status_text, status_circle

def on_animation_complete():
    complete_circle.set_color('green')
    fig.canvas.draw_idle()

ani = animation.FuncAnimation(fig, update, frames=len(data), interval=100, blit=False, repeat=False)

## Can invert axis if wish
#ax.invert_yaxis()
#ax.invert_xaxis()

print(len(data))
plt.show()

# ## Save to MP4 (requires ffmpeg)
# ani.save('robot_playback.mp4', writer='ffmpeg', fps=10)


