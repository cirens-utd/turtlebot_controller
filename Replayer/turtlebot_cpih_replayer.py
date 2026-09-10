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

class TukeyCenterPointPlugin:
    def __init__(self, color=None, alpha=0.25):
        self.imprecision = 0.01
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
        self.title = "Tukey Graph"
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

        self.patches = None
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
        # Zero Line
        line_num = 0
        self.mode_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Tukey Mode: ', fontsize=self.fontsize, ha='left', va='top')
        self.mode_text = self.fig.text(self.start_line_x + 0.11, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # First line
        line_num = line_num + 1
        self.trust_label = self.fig.text(self.start_line_x - 0.01, self.start_line_y - self.delta_line_y*line_num, 'Trust Mode: ', fontsize=self.fontsize, ha='left', va='top')
        self.trust_text = self.fig.text(self.start_line_x + 0.11, self.start_line_y - self.delta_line_y*line_num, 'None', fontsize=self.fontsize, ha='left', va='top')

        # # Time Graph
        self.ax.set_ylim(0, 5)
        self.ax.set_title(self.title)
        self.ax.margins(y=0)

        self.time_x = []
        self.time_tukey_depth = []
        self.time_center_depth = []

        self.time_line_tukey, = self.ax.plot([], [], color='blue', label='Tukey Depth')
        self.time_line_center, = self.ax.plot([], [], color='red', label='Center Depth')
        self.ax.legend(loc='upper right')

        plt.show(block=False)


    def update(self, viz, frame):

        if self.first_frame:
            self.first_frame = False
            print(f"Using Mode: {viz.data.safe_point_mode[0]} and Trust: {viz.data.self_trust[0]}")

            # Finidng countour we should stay inside of. This excluded the adversary (My_number > 10)
            my_number = int(viz.data.my_name[0][5:])
            boundary_pts = []
            if my_number < 10:
                # This robot
                boundary_pts.append([viz.data.x_vals[frame], viz.data.y_vals[frame]])

                # # neighbors
                # for _, pose in viz.data.neighbor_poses[frame].items():
                #     if pose['in_neighborhood']:
                #         boundary_pts.append([pose["x"], pose["y"]])
                #     # else:
                #     #     boundary_pts.append([pose["x"], pose["y"]])
                for name, pose in viz.data.neighbor_position[frame].items():
                    if int(name) < 10:
                        boundary_pts.append(pose)

                boundary_pts = np.array(boundary_pts)

                best_order = None
                best_area = -1

                for order in permutations(range(len(boundary_pts))):
                    poly = Polygon([boundary_pts[i] for i in order])

                    if poly.is_valid and poly.area > best_area:
                        best_area = poly.area
                        best_order = order

                best_order = list(best_order)

                self.boundary_pts = np.array([boundary_pts[i] for i in best_order])

        # Only recompute if frame changed
        if frame == self.last_frame:
            return
        self.last_frame = frame

        self.draw_area(np.array(viz.data.safe_area[frame]), viz)

        viz.fig.canvas.draw_idle()

        # draw second window
        if self.fig is None:
            self.setup()

        self.mode_text.set_text(str(viz.data.safe_point_mode[frame]))
        self.trust_text.set_text(str(viz.data.self_trust[frame]))

        self.time_x.append(frame)
        self.time_tukey_depth.append(viz.data.tukey_depth[frame])
        self.time_center_depth.append(viz.data.center_depth[frame])

        self.time_x = self.time_x[-1*self.window:]
        self.time_tukey_depth = self.time_tukey_depth[-1*self.window:]
        self.time_center_depth = self.time_center_depth[-1*self.window:]

        self.time_line_tukey.set_data(self.time_x, self.time_tukey_depth)
        self.time_line_center.set_data(self.time_x, self.time_center_depth)

        # Rescale dynamically
        # X axis
        if frame > self.window:
            self.ax.set_xlim(frame - self.window, frame)
        else:
            self.ax.set_xlim(0, self.window)
        
        # Y axis
        if len(self.time_tukey_depth) > 1:
            ymin = min(min(self.time_tukey_depth, self.time_center_depth))
            ymax = max(max(self.time_tukey_depth, self.time_center_depth))
            
            padding = 0.1 * (ymax - ymin + 1e-6)
            self.ax.set_ylim(ymin - padding, ymax + padding)

        # if frame > 108:
        #     self.my_testing(viz, frame)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        return self.time_line_tukey, self.mode_text, self.trust_text

    def my_testing(self, viz, frame):
        X = np.zeros((len(viz.data.neighbor_position[frame])+1,2))
        i = 1
        X[0] = [viz.data.x_vals[frame], viz.data.y_vals[frame]]
        for name, neighbor in viz.data.neighbor_position[frame].items():
            X[i] = np.array(neighbor) 
            # print("X[",i,"]: ",X[i])
            i = i+1
        Bx = self.getImprecisionRegions(X, viz.data.imprecision[frame])

        centerpoint = True
        tc = TukeyContour(X, 0, centerpoint=centerpoint, mode=viz.data.self_trust[frame])
        # self._safe_area = tc.median_contour.tolist()
        final_poly, tukey_depth, center_depth = SelfTukeyMed(X, 0, centerpoint)
        # self._safe_area = final_poly.tolist()
        sp = SafePoint()
        centroid, region, tukey_depth, centerpoint_depth = sp.CPIH_Fast_Safepoint(Bx, 0, X[0], mode=viz.data.self_trust[frame])
        # self._safe_area = np.array(region.exterior.coords).tolist()

        # self.draw_area(np.array(viz.data.safe_area[frame]), viz)
        plot_safepoint(
            X,
            Bx,
            Xi=0,
            centerpoint=centerpoint,
            mode=viz.data.self_trust[frame]
        )
        pdb.set_trace()

    def getImprecisionRegions(self, X,imp):
        n = len(X)
        Bx= np.zeros((n,4,2))
        for i in range(len(X)):
            Bx[i,0,:]= [X[i][0]-imp, X[i][1]+imp]
            Bx[i,1,:]= [X[i][0]+imp, X[i][1]+imp]
            Bx[i,2,:]= [X[i][0]+imp, X[i][1]-imp]
            Bx[i,3,:]= [X[i][0]-imp, X[i][1]-imp]
        return(Bx)

    def contour_to_poly(self, contour):
        if contour is None or len(contour) < 3:
            return None

        contour = np.asarray(contour)

        if not np.allclose(contour[0], contour[-1]):
            contour = np.vstack([contour, contour[0]])

        poly = Polygon(contour)

        if not poly.is_valid:
            poly = poly.buffer(0)

        return poly

    def draw_area(self, contour, viz):

        # remove old patch
        if self.patches is not None:
            self.patches.remove()
            self.patches = None

        if contour is None or len(contour) < 3:
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

        # # If you want to test the point it should be going to
        # contour_poly = self.contour_to_poly(contour)
        # if False:
        #     if str(i) not in self.centroid_points:
        #         self.centroid_points[str(i)] = viz.ax.plot(
        #             contour_poly.centroid.y,
        #             contour_poly.centroid.x,
        #             marker="x",
        #             markersize=10,
        #             color=self.colors[i+1 % len(self.colors)],
        #             zorder=7
        #         )[0]
        #     else:
        #         self.centroid_points[str(i)].set_data([contour_poly.centroid.y], [contour_poly.centroid.x])
        # else:
        #     temp_centroid = np.mean(contour, axis=0)
        #     if str(i) not in self.centroid_points:
        #         self.centroid_points[str(i)] = viz.ax.plot(
        #             temp_centroid[1],
        #             temp_centroid[0],
        #             marker="x",
        #             markersize=10,
        #             color=self.colors[i+1 % len(self.colors)],
        #             zorder=7
        #         )[0]
        #     else:
        #         self.centroid_points[str(i)].set_data([temp_centroid[1]], [temp_centroid[0]])

        viz.ax.add_patch(patch)
        self.patches = patch

        # adding interanl boundary area
        if len(self.boundary_pts):
            poly_xy = np.column_stack([self.boundary_pts[:, 1], self.boundary_pts[:, 0]])

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
    replayVisual.replay_schema.add("safe_area")
    replayVisual.replay_schema.add("tukey_depth")
    replayVisual.replay_schema.add("center_depth")
    replayVisual.replay_schema.add("self_trust")
    replayVisual.replay_schema.add("safe_point_mode")
    replayVisual.replay_schema.add("imprecision")
    replayVisual.frame_rate = 10    # 10 frames is "Real Time"
    # replayVisual.xmin = -110
    replayVisual.load_data()
    replayVisual.setup()

    # voronoi_plugin = TukeyCenterPointPlugin(trust=script_args.trust, mode=script_args.mode)
    voronoi_plugin = TukeyCenterPointPlugin()
    replayVisual.add_plugin(voronoi_plugin)

    replayVisual.start_animation()
    replayVisual.run()

if __name__ == '__main__':
    main()