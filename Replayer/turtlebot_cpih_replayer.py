from scipy.spatial import Voronoi
from shapely.geometry import Polygon, box, LineString, Point
from turtlebot_replayer import ReplayVisualizer
from TukeyMedian import TukeyContour, SafePoint
import numpy as np
import matplotlib.patches as patches
import argparse
import pdb

class TukeyCenterPointPlugin:
    def __init__(self, trust=1, mode=0, color=None, alpha=0.25):
        self.trust = int(trust)
        self.mode = int(mode)
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

        self.patch = None
        self.last_frame = -1

        self.centroid_points = {}

        print(f"Using Mode: {mode} and Trust: {trust}")


    def update(self, viz, frame):

        # Only recompute if frame changed
        if frame == self.last_frame:
            return
        self.last_frame = frame

        # -----------------------------
        # 1. Collect points
        # -----------------------------
        pts = []

        # robot
        pts.append([viz.x_vals[frame], viz.y_vals[frame]])

        # neighbors
        for _, pose in viz.neighbor_poses[frame].items():
            if pose['in_neighborhood']:
                pts.append([pose["x"], pose["y"]])
            # else:
            #     pts.append([pose["x"], pose["y"]])

        pts = np.array(pts)

        if len(pts) < 3:
            return

        if not hasattr(self, "patches"):
            self.patches = {}

        for i in range(len(pts)):
            if self.mode == 0:
                self.centerpoint(i, pts, viz)
            elif self.mode == 1:
                self.safepoint(i, pts, viz)

        viz.fig.canvas.draw_idle()

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

    def centerpoint(self, i, pts, viz):
        contour = TukeyContour(
                input_points=pts,
                Xi=i,
                mode=self.trust,
                verbose=False
            ).median_contour

        # remove old patch
        if i in self.patches and self.patches[i]:
            self.patches[i].remove()
            self.patches[i] = None

        if contour is None or len(contour) < 3:
            return

        poly_xy = np.column_stack([contour[:, 1], contour[:, 0]])

        patch = patches.Polygon(
            poly_xy,
            closed=True,
            facecolor=self.colors[i % len(self.colors)],
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
        self.patches[i] = patch
           
    def safepoint(self, i, pts, viz):
        Bx = self.getImprecisionRegions(pts,self.imprecision)

        sp = SafePoint()
        centroid, region = sp.CPIH_Fast_Safepoint(Bx, i, pts[i], mode=self.trust)

        # remove old patch
        if i in self.patches and self.patches[i]:
            self.patches[i].remove()
            self.patches[i] = None

        if  region is None or region.is_empty:
            return

        # If MultiPolygon → take largest
        if region.geom_type == "MultiPolygon":
            region = max(region.geoms, key=lambda g: g.area)

        # If not a polygon, skip
        if region.geom_type != "Polygon":
            return

        # -----------------------------
        # Extract coordinates
        # -----------------------------
        coords = np.array(region.exterior.coords)

        if len(coords) < 3:
            return

        poly_xy = np.column_stack([coords[:, 1], coords[:, 0]])

        # -----------------------------
        # Create patch
        # -----------------------------
        patch = patches.Polygon(
            poly_xy,
            closed=True,
            facecolor=self.colors[i % len(self.colors)],
            edgecolor="black",
            linewidth=1.0,
            alpha=self.alpha,
            zorder=1
        )

        # # -----------------------------
        # # Optional: centroid plotting
        # # -----------------------------
        # centroid = region.centroid

        # if str(i) not in self.centroid_points:
        #     self.centroid_points[str(i)] = viz.ax.plot(
        #         centroid.x,
        #         centroid.y,
        #         marker="x",
        #         markersize=10,
        #         color=self.colors[(i + 1) % len(self.colors)],
        #         zorder=7
        #     )[0]
        # else:
        #     self.centroid_points[str(i)].set_data([centroid.x], [centroid.y])

        # -----------------------------
        # Add patch
        # -----------------------------
        viz.ax.add_patch(patch)
        self.patches[i] = patch


    def getImprecisionRegions(self, X,imp):
        n = len(X)
        Bx= np.zeros((n,4,2))
        for i in range(len(X)):
            Bx[i,0,:]= [X[i][0]-imp, X[i][1]+imp]
            Bx[i,1,:]= [X[i][0]+imp, X[i][1]+imp]
            Bx[i,2,:]= [X[i][0]+imp, X[i][1]-imp]
            Bx[i,3,:]= [X[i][0]-imp, X[i][1]-imp]
        return(Bx)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--save", default=False, action="store_true", help="Save MP4")
    parser.add_argument("-p", "--play", default=True, action="store_false", help="Set to not show graph")
    parser.add_argument("-f", "--filename", default="Example", type=str, help="Name of MP4 file without .mp4")
    parser.add_argument("-b", "--beauty", default=False, action="store_true", help="Save Pretty Json")
    parser.add_argument("-t", "--trust", default=0, help="Set the Trust Mode")
    parser.add_argument("-m", "--mode", default=0, help="Set the TukeyMode")
    script_args = parser.parse_args()

    replayVisual = ReplayVisualizer(script_args.play, script_args.save, script_args.filename, script_args.beauty)
    replayVisual.frame_rate = 10    # 10 frames is "Real Time"
    # replayVisual.xmin = -110
    replayVisual.load_data()
    replayVisual.setup()

    voronoi_plugin = TukeyCenterPointPlugin(trust=script_args.trust, mode=script_args.mode)
    replayVisual.add_plugin(voronoi_plugin)

    replayVisual.start_animation()
    replayVisual.run()

if __name__ == '__main__':
    main()