#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
from agent_control.coverage import Coverage
from agent_control.testme import TestMe

from scipy.spatial import Voronoi
from shapely.geometry import Polygon, LineString, Point, box
import matplotlib.animation as animation
import matplotlib.patches as patches
import argparse
import datetime
import traceback
import itertools
import sys
import os
import threading

import pdb

# getting replayer functions
script_dir = os.path.dirname(os.path.abspath(__file__))
replayer_path = os.path.abspath(os.path.join(script_dir, '../../../'))
sys.path.append(replayer_path)

from Replayer import turtlebot_replayer

def start_animation():
    # global turtlebot_replayer.ani
    # global turtlebot_replayer.fig
    global update

    turtlebot_replayer.ani = animation.FuncAnimation(
        turtlebot_replayer.fig, 
        update, 
        init_func=turtlebot_replayer.init,
        frames=itertools.count(), 
        interval=100, 
        blit=False, repeat=False)
    turtlebot_replayer.fig.canvas.draw_idle()

def update(frame):
    # global turtlebot_replayer.goal_marker_x
    # global turtlebot_replayer.goal_attempt_marker_x
    # global turtlebot_replayer.aspect
    # global turtlebot_replayer.trail
    # global turtlebot_replayer.trail_coords
    # global turtlebot_replayer.robot_radius
    # global turtlebot_replayer.robot_marker_arrow
    # global turtlebot_replayer.goal_marker_anlge

    if type(my_robot.pose) == type(None):
        my_robot.get_logger().info("Pose not updated yet")
        return

    # update my position
    x,y = my_robot.pose['pose']['position']['x'], my_robot.pose['pose']['position']['y']

    ori = my_robot.pose['pose']['orientation']
    qx, qy, qz, qw = ori['x'], ori['y'], ori['z'], ori['w']
    yaw = np.remainder((np.arctan2(2 * (qw * qz + qx * qy),1 - 2 * (qy * qy + qz * qz)) + np.pi) , 2 * np.pi) + np.pi

    turtlebot_replayer.robot_marker_circle.set_center((y, x))    # note they are backwards
    turtlebot_replayer.robot_marker_circle.set_color('lightgreen' if my_robot.robot_ready else 'mistyrose')

    turtlebot_replayer.robot_marker_arrow.remove()
    arrow_length = 1.2 * turtlebot_replayer.robot_radius
    arrow_dx = arrow_length * np.cos(yaw)
    arrow_dy = arrow_length * np.sin(yaw)
    turtlebot_replayer.robot_marker_arrow = patches.FancyArrowPatch((y, x), (y+arrow_dy, x+arrow_dx), 
                                                arrowstyle='->',
                                                mutation_scale=20, color='blue',
                                                linewidth=2, zorder=11)
    turtlebot_replayer.ax.add_patch(turtlebot_replayer.robot_marker_arrow)

    # update neighbor position

    for name, full_pose in my_robot.neighbor_poses.items():
        pose = full_pose['pose']['position']
        ori = full_pose['pose']['orientation']
        qx, qy, qz, qw = ori['x'], ori['y'], ori['z'], ori['w']
        yaw = np.remainder((np.arctan2(2 * (qw * qz + qx * qy),1 - 2 * (qy * qy + qz * qz)) + np.pi) , 2 * np.pi)

        turtlebot_replayer.neighbor_marker[name].set_center((pose['y'], pose['x']))

        turtlebot_replayer.neighbor_arrow[name].remove()
        neighbor_orientation = yaw + np.pi
        arrow_dx = arrow_length * np.cos(neighbor_orientation)
        arrow_dy = arrow_length * np.sin(neighbor_orientation)
        turtlebot_replayer.neighbor_arrow[name] = patches.FancyArrowPatch((pose['y'], pose['x']), (pose['y']+arrow_dy, pose['x']+arrow_dx), 
                                                    arrowstyle='->',
                                                    mutation_scale=20, color='darkorange',
                                                    linewidth=2, zorder=9)
        turtlebot_replayer.ax.add_patch(turtlebot_replayer.neighbor_arrow[name])

    # updating goal location 
    if my_robot.destination_reached:
        if turtlebot_replayer.goal_marker_x in turtlebot_replayer.ax.lines:
            turtlebot_replayer.goal_marker_x.remove()
        goal_y, goal_x = my_robot.desired_location[1], my_robot.desired_location[0]
        goal_ori = my_robot.desired_angle + np.pi
        arrow_dx = arrow_length * np.cos(goal_ori)
        arrow_dy = arrow_length * np.sin(goal_ori)
        turtlebot_replayer.goal_marker_anlge.remove()
        turtlebot_replayer.goal_marker_anlge = patches.FancyArrowPatch((goal_y, goal_x), (goal_y+arrow_dy, goal_x+arrow_dx), 
                                            arrowstyle='->',
                                            mutation_scale=20, color='#FF10F0',
                                            linewidth=2, zorder=12)
        turtlebot_replayer.ax.add_patch(turtlebot_replayer.goal_marker_anlge)

    else:
        turtlebot_replayer.goal_marker_anlge.remove()
        turtlebot_replayer.goal_marker_anlge = patches.FancyArrowPatch((-100, -100), (0, -100), 
                                            arrowstyle='->',
                                            mutation_scale=20, color='#FF10F0',
                                            linewidth=2, zorder=12)
        turtlebot_replayer.ax.add_patch(turtlebot_replayer.goal_marker_anlge)
        if turtlebot_replayer.goal_marker_x not in turtlebot_replayer.ax.lines:
            turtlebot_replayer.goal_marker_x = turtlebot_replayer.ax.plot(-100,-100, marker='x', color='#FF10F0', markersize=15, zorder=11)[0]
        if type(my_robot.desired_location) != type(None):
            turtlebot_replayer.goal_marker_x.set_data([my_robot.desired_location[1]], [my_robot.desired_location[0]])

    if type(my_robot._attempted_desired_location) != type(None):
        turtlebot_replayer.goal_attempt_marker_x.set_data([my_robot._attempted_desired_location[1]], [my_robot._attempted_desired_location[0]])

    # updating status variables
    turtlebot_replayer.status_text.set_text(my_robot.robot_status)
    turtlebot_replayer.ready_text.set_text('Robot Ready' if my_robot.robot_ready else 'Robot Not Ready')
    turtlebot_replayer.ready_text.set_color('green' if my_robot.robot_ready else 'red')
    turtlebot_replayer.ready_circle.set_color('green' if my_robot.robot_ready else 'red')
    turtlebot_replayer.pos_started_text.set_text('Position Obtained' if my_robot._position_started else 'Position Not Obtrained')
    turtlebot_replayer.pos_started_text.set_color('green' if my_robot._position_started else 'red')
    turtlebot_replayer.pos_started_circle.set_color('green' if my_robot._position_started else 'red')
    turtlebot_replayer.neighbors_started_text.set_text('Neighbor Position Obtained' if my_robot._neighbors_started else 'Neighbor Position Not Obtrained')
    turtlebot_replayer.neighbors_started_text.set_color('green' if my_robot._neighbors_started else 'red')
    turtlebot_replayer.neighbors_started_circle.set_color('green' if my_robot._neighbors_started else 'red')
    turtlebot_replayer.lidar_started_text.set_text('Lidar Obtained' if my_robot._lidar_started else 'Lidar Not Obtrained')
    turtlebot_replayer.lidar_started_text.set_color('green' if my_robot._lidar_started else 'red')
    turtlebot_replayer.lidar_started_circle.set_color('green' if my_robot._lidar_started else 'red')
    turtlebot_replayer.robot_moving_text.set_text('Robot Moving' if my_robot.robot_moving else 'Robot Not Moving')
    turtlebot_replayer.robot_moving_text.set_color('green' if my_robot.robot_moving else 'red')
    turtlebot_replayer.robot_moving_circle.set_color('green' if my_robot.robot_moving else 'red')
    turtlebot_replayer.movement_restricted_text.set_text('Movement Restricted' if my_robot._robot_restricted_movement else 'Movement Not Restricted')
    turtlebot_replayer.movement_restricted_text.set_color('green' if my_robot._robot_restricted_movement else 'red')
    turtlebot_replayer.movement_restricted_circle.set_color('green' if my_robot._robot_restricted_movement else 'red')
    turtlebot_replayer.path_obstructed_text.set_text('Path Obstructed' if my_robot.path_obstructed else 'Path Not Obstructed')
    turtlebot_replayer.path_obstructed_text.set_color('green' if my_robot.path_obstructed else 'red')
    turtlebot_replayer.path_obstructed_circle.set_color('green' if my_robot.path_obstructed else 'red')
    turtlebot_replayer.laser_obstructed_text.set_text('Laser Obstructed' if my_robot.path_obstructed_laser else 'Laser Not Obstructed')
    turtlebot_replayer.laser_obstructed_text.set_color('green' if my_robot.path_obstructed_laser else 'red')
    turtlebot_replayer.laser_obstructed_circle.set_color('green' if my_robot.path_obstructed_laser else 'red')
    turtlebot_replayer.neighbor_obstructed_text.set_text('Neighbor Obstructed' if my_robot.path_obstructed_neighbor else 'Neighbor Not Obstructed')
    turtlebot_replayer.neighbor_obstructed_text.set_color('green' if my_robot.path_obstructed_neighbor else 'red')
    turtlebot_replayer.neighbor_obstructed_circle.set_color('green' if my_robot.path_obstructed_neighbor else 'red')
    turtlebot_replayer.laser_avoid_error_text.set_text('Laser Avoid Error' if my_robot.laser_avoid_error else 'No Laser Avoid Error')
    turtlebot_replayer.laser_avoid_error_text.set_color('green' if my_robot.laser_avoid_error else 'red')
    turtlebot_replayer.laser_avoid_error_circle.set_color('green' if my_robot.laser_avoid_error else 'red')
    turtlebot_replayer.desired_heading_text.set_text('Desired Heading Reached' if my_robot.desired_heading else 'Desired Heading Not Reached')
    turtlebot_replayer.desired_heading_text.set_color('green' if my_robot.desired_heading else 'red')
    turtlebot_replayer.desired_heading_circle.set_color('green' if my_robot.desired_heading else 'red')
    turtlebot_replayer.destination_reached_text.set_text('Destination Reached' if my_robot.destination_reached else 'Destination Not Reached')
    turtlebot_replayer.destination_reached_text.set_color('green' if my_robot.destination_reached else 'red')
    turtlebot_replayer.destination_reached_circle.set_color('green' if my_robot.destination_reached else 'red')
    turtlebot_replayer.motion_complete_text.set_text('Motion Completed' if my_robot.motion_complete else 'Motion Not Completed')
    turtlebot_replayer.motion_complete_text.set_color('green' if my_robot.motion_complete else 'red')
    turtlebot_replayer.motion_complete_circle.set_color('green' if my_robot.motion_complete else 'red')
    turtlebot_replayer.neighbor_complete_text.set_text('Neighbors Completed' if my_robot.neighbors_complete else 'Neighbors Not Completed')
    turtlebot_replayer.neighbor_complete_text.set_color('green' if my_robot.neighbors_complete else 'red')
    turtlebot_replayer.neighbor_complete_circle.set_color('green' if my_robot.neighbors_complete else 'red')

    # update tolerances
    turtlebot_replayer.destination_tolerance_text.set_text(f"Destination Tolerance: {my_robot._destination_tolerance}")
    turtlebot_replayer.angle_tolerance_text.set_text(f"Angle Tolerance: {my_robot._angle_tolerance}")
    if type(my_robot.desired_location) != type(None):
        new_goal_radius = my_robot._destination_tolerance
        turtlebot_replayer.goal_radius.set_center((my_robot.desired_location[1], my_robot.desired_location[0]))
        turtlebot_replayer.goal_radius.radius = new_goal_radius

    # update LED Ring
    if type(my_robot.led_light_state) != type(None):
        led_colors = []
        for led in my_robot.led_light_state['leds']:
            led_colors.append((led['red']/255, led['green']/255, led['blue']/255))
        turtlebot_replayer.update_led_ring(led_colors)

    turtlebot_replayer.trail_coords.append((y, x))
    if len(turtlebot_replayer.trail_coords) > 100:
        turtlebot_replayer.trail_coords.pop(0)
    
    if turtlebot_replayer.trail not in turtlebot_replayer.ax.lines:
        turtlebot_replayer.trail, = turtlebot_replayer.ax.plot([], [], 'o-', color='lightblue', markersize=4, zorder=5)
    turtlebot_replayer.trail.set_data(*zip(*turtlebot_replayer.trail_coords))

    return turtlebot_replayer.robot_marker_circle
        
def spin_node():
    global my_robot
    rclpy.spin(my_robot)

def main(args=None):
    global my_robot
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=3
    ## ros2 run agent_control coverage.py -i 1 -s -n 2 3 -p 0 0 0 2 2 2 2 0

    '''
    New modules needed:
    - scipy
    - shapely
    '''

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("-p", "--perimeter", nargs='+', type=float, help="List of coordinates defining the ploygon (e.g. x1 y1 x2 y2 ...)")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    if script_args.perimeter:
        points = [(script_args.perimeter[i], script_args.perimeter[i+1]) for i in range(0, len(script_args.perimeter), 2)]
    else:
        points = [(-2.9, -5),(-2.9, 4),(2.9, 4), (2.9, -5)]

    try:
        rclpy.init(args=args)
        my_robot = TestMe(int(script_args.index), np.array(script_args.neighbor), #perimeter=points,
            sim=script_args.sim, viewer=True,
            restricted_area=True, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
        

        threading.Thread(target=spin_node, daemon= True).start()


        turtlebot_replayer.neighbor_info = my_robot.neighbor_poses
        turtlebot_replayer.setup_graph()
        turtlebot_replayer.restart_button.ax.remove()
        turtlebot_replayer.ax.invert_xaxis()
        start_animation()
        turtlebot_replayer.plt.show()
    except Exception as e:
        traceback.print_exc()
    finally:
        my_robot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()