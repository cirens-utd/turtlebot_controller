import matplotlib.pyplot as plt
from os import listdir, remove, rmdir, path, getcwd
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, box, LineString, Point
from itertools import permutations
from turtlebot_replayer import ReplayVisualizer
from TukeyMedian import TukeyContour, SafePoint, SelfTukeyMed
from draw_safepoint import plot_safepoint
import numpy as np
import matplotlib.patches as patches
import argparse
import pdb

new_start_path = path.abspath(path.join(getcwd(), "..", "Replays","CPIH_Paper","AdversaryStatic"))

class TukeyCenterPointAdversaryPlugin:
    def __init__(self, color=None, alpha=0.25):
        self.colors = color or [
                                    "#4C78A8",  # blue
                                    "#F58518",  # orange
                                    "#54A24B",  # green
                                    "#E45756",  # red
                                    "#B279A2",  # purple
                                    "#9D755D",  # brown
                                    "#FF9DA6",  # light pink
                                    "#79706E",  # dark gray
                                    "#BAB0AC",  # light gray
                                ]
        self.alpha = alpha

        # setting up secondary window
        self.title = "Adversary Data"
        self.size = (10,6) 
        self.fig = None
        self. ax = None
        self.start_line_x = 0.02
        self.start_line_y = 0.95
        self.delta_line_x = 0.1
        self.delta_line_y = 0.05
        self.x_indent = 0.015
        self.fontsize = 12

        self.window = 500

        self.patch = None
        self.boundary_pts = []
        self.first_frame = True
        self.last_frame = -1

        self.centroid_points = {}

    def setup(self):
        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=self.size)  # (x, y) x inches wide and y inches tall
        self.fig.subplots_adjust(left=0.35)           # leave 35% of area on left
        self.ax.set_aspect('auto')

        ## Information on Left
        indent = 0.21

        # Zero Line
        line_num = 0
        self.score_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Score: ', fontsize=self.fontsize, ha='left', va='top')
        self.score_text = self.fig.text(self.start_line_x + indent, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # First line
        line_num = line_num + 1
        self.lastScore_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Last Score: ', fontsize=self.fontsize, ha='left', va='top')
        self.lastScore_text = self.fig.text(self.start_line_x + indent, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # Second line
        line_num = line_num + 1
        self.target_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Target: ', fontsize=self.fontsize, ha='left', va='top')
        self.target_text = self.fig.text(self.start_line_x + indent, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # Third line
        line_num = line_num + 1
        self.lastTarget_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Last Target: ', fontsize=self.fontsize, ha='left', va='top')
        self.lastTarget_text = self.fig.text(self.start_line_x + indent, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # Forth line
        line_num = line_num + 1
        self.jumpBlocked_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Jump Blocked: ', fontsize=self.fontsize, ha='left', va='top')
        self.jumpBlocked_text = self.fig.text(self.start_line_x + indent, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # Fifth line
        line_num = line_num + 1
        self.attackNumber_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'NeighborsAttacked: ', fontsize=self.fontsize, ha='left', va='top')
        self.attackNumber_text = self.fig.text(self.start_line_x + indent, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # Sixth line
        line_num = line_num + 1
        self.big_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'BIG: ', fontsize=self.fontsize, ha='left', va='top')
        self.big_text = self.fig.text(self.start_line_x + indent, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # # Time Graph
        self.ax.set_ylim(0, 5)
        self.ax.set_title(self.title)
        self.ax.margins(y=0)

        # self.time_x = []
        # self.time_tukey_depth = []
        # self.time_center_depth = []

        # self.time_line_tukey, = self.ax.plot([], [], color='blue', label='Tukey Depth')
        # self.time_line_center, = self.ax.plot([], [], color='red', label='Center Depth')
        # self.ax.legend(loc='upper right')

        plt.show(block=False)

    def update(self, viz, frame):

        # Only recompute if frame changed
        if frame == self.last_frame:
            return
        self.last_frame = frame

        if not hasattr(self, "patches"):
            self.patches = None

        self.draw_area(np.array(viz.data.hull_poly[frame]), viz)
        self.draw_lines(np.array(viz.data.wedge_set_lines[frame]), viz.data.big_box[frame], viz)
        # Draw Target
        # Draw Apex Values

        viz.fig.canvas.draw_idle()

        # draw second window
        if self.fig is None:
            self.setup()

        self.score_text.set_text(str(np.round(viz.data.score[frame], 2)))
        self.lastScore_text.set_text(str(np.round(viz.data.last_score[frame],2)))
        self.target_text.set_text(str(np.round(np.array(viz.data.target[frame]), 2)))
        self.lastTarget_text.set_text(str(np.round(np.array(viz.data.last_target[frame]), 2)))
        self.jumpBlocked_text.set_text(str(viz.data.jump_blocked[frame]))
        self.attackNumber_text.set_text(str(viz.data.num_compromised[frame]))
        self.big_text.set_text(str(viz.data.big_box[frame]))

        # self.time_x.append(frame)
        # self.time_tukey_depth.append(viz.data.tukey_depth[frame])
        # self.time_center_depth.append(viz.data.center_depth[frame])

        # self.time_x = self.time_x[-1*self.window:]
        # self.time_tukey_depth = self.time_tukey_depth[-1*self.window:]
        # self.time_center_depth = self.time_center_depth[-1*self.window:]

        # self.time_line_tukey.set_data(self.time_x, self.time_tukey_depth)
        # self.time_line_center.set_data(self.time_x, self.time_center_depth)

        # # Rescale dynamically
        # # X axis
        # if frame > self.window:
        #     self.ax.set_xlim(frame - self.window, frame)
        # else:
        #     self.ax.set_xlim(0, self.window)
        
        # # Y axis
        # if len(self.time_tukey_depth) > 1:
        #     ymin = min(min(self.time_tukey_depth, self.time_center_depth))
        #     ymax = max(max(self.time_tukey_depth, self.time_center_depth))
            
        #     padding = 0.1 * (ymax - ymin + 1e-6)
        #     self.ax.set_ylim(ymin - padding, ymax + padding)

        # if frame > 108:
        #     self.my_testing(viz, frame)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        return self.score_text

    def draw_area(self, contour, viz):

        # remove old patch
        if self.patches is not None:
            self.patches.remove()
            self.patches = None

        if type(contour) == type(None) or len(contour) < 3:
            return

        poly_xy = np.column_stack([contour[:, 1], contour[:, 0]])

        patch = patches.Polygon(
            poly_xy,
            closed=True,
            facecolor=self.colors[0 % len(self.colors)],
            edgecolor="black",
            linewidth=1.0,
            alpha=self.alpha,
            zorder=1
        )

        viz.ax.add_patch(patch)
        self.patches = patch
    
    def draw_lines(self, line_set, big, viz):
        # line_set = [
        #     [
        #         [x,y],
        #         [x,y]
        #     ]
        # ]
        return

    def restart(self, event):
        self.ax.set_ylim(0, 5)

        self.time_x = []
        self.time_tukey_depth = []

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--save", default=False, action="store_true", help="Save MP4")
    parser.add_argument("-p", "--play", default=True, action="store_false", help="Set to not show graph")
    parser.add_argument("-f", "--filename", default="Example", type=str, help="Name of MP4 file without .mp4")
    parser.add_argument("-b", "--beauty", default=False, action="store_true", help="Save Pretty Json")
    # parser.add_argument("-t", "--trust", default=0, help="Set the Trust Mode")
    # parser.add_argument("-m", "--mode", default=0, help="Set the TukeyMode")
    script_args = parser.parse_args()

    replayVisual = ReplayVisualizer(script_args.play, script_args.save, script_args.filename, script_args.beauty)
    replayVisual.start_path = new_start_path
    replayVisual.replay_schema.add("target")
    replayVisual.replay_schema.add("score")
    replayVisual.replay_schema.add("last_target")
    replayVisual.replay_schema.add("last_score")
    replayVisual.replay_schema.add("num_compromised")
    replayVisual.replay_schema.add("targetNeighborhoods")
    replayVisual.replay_schema.add("targetNeighborhoodStates")
    replayVisual.replay_schema.add("wedge_set_lines")
    replayVisual.replay_schema.add("wedge_set_apex")
    replayVisual.replay_schema.add("hull_poly")
    replayVisual.replay_schema.add("jump_blocked")
    replayVisual.replay_schema.add("big_box")
    replayVisual.frame_rate = 10    # 10 frames is "Real Time"
    # replayVisual.xmin = -110
    replayVisual.load_data()
    replayVisual.setup()

    # voronoi_plugin = TukeyCenterPointPlugin(trust=script_args.trust, mode=script_args.mode)
    voronoi_plugin = TukeyCenterPointAdversaryPlugin()
    replayVisual.add_plugin(voronoi_plugin)

    replayVisual.start_animation()
    replayVisual.run()

if __name__ == '__main__':
    main()