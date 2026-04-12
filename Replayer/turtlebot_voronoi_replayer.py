from scipy.spatial import Voronoi
from shapely.geometry import Polygon, box, LineString, Point
from turtlebot_replayer import ReplayVisualizer
import numpy as np
import matplotlib.patches as patches
import argparse
import pdb

class BoundedVoronoiPlugin:
    def __init__(self, bounds=None, alpha=0.2):
        self.bounds = bounds or box(-10, -10, 10, 10)
        self.alpha = alpha
        self.patches = []
        self.last_frame = -1

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

        pts = np.array(pts)

        if len(pts) < 2:
            return

        # -----------------------------
        # 2. Compute Voronoi
        # -----------------------------
        vor = Voronoi(pts)

        # -----------------------------
        # 3. Clear old patches
        # -----------------------------
        for p in self.patches:
            p.remove()
        self.patches = []

        # -----------------------------
        # 4. Draw bounded regions
        # -----------------------------
        for i in range(len(pts)):
            region_poly = self.bounded_voronoi_region(vor, i, self.bounds)

            if type(region_poly) != type(None):
                x,y = region_poly.exterior.xy

                if not region_poly or region_poly.is_empty:
                    continue

                x, y = region_poly.exterior.xy

                patch = patches.Polygon(
                    np.column_stack([y, x]),
                    closed=True,
                    alpha=self.alpha,
                    edgecolor="black",
                    facecolor="cyan",
                    zorder=1
                )

                viz.ax.add_patch(patch)
                self.patches.append(patch)

        viz.fig.canvas.draw_idle()

    def bounded_voronoi_region(self, vor, point_idx, bounding_shape, far_enough=100):
        region_idx = vor.point_region[point_idx]
        region = vor.regions[region_idx]
        if -1 in region or len(region) == 0:
            # Region is infinite, reconstruct polygon with ridge directions
            point = vor.points[point_idx]
            ridges = [r for r in vor.ridge_points if point_idx in r]
            lines = []
            for (p1_idx, p2_idx), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
                if point_idx not in (p1_idx, p2_idx):
                    continue
                if v1 == -1 or v2 == -1:
                    # Get the two original points that generated this ridge
                    p1 = vor.points[p1_idx]
                    p2 = vor.points[p2_idx]

                    # Compute midpoint of p1 and p2 (for visualizing or anchoring)
                    midpoint = (p1 + p2) / 2

                    # Compute the vector between them
                    line_vec = p2 - p1

                    # Compute the normal (perpendicular vector)
                    direction = np.array([-line_vec[1], line_vec[0]])
                    normal = direction / np.linalg.norm(direction)  # Normalize

                    # Choose Direction
                    finite_vertex = vor.vertices[v1 if v1 != -1 else v2]
                    to_vertex = finite_vertex - midpoint

                    # If dot product is negative, we are going the wrong way
                    if np.dot(to_vertex, normal) > 0:
                        normal *= -1 
                    
                    far_point = finite_vertex + far_enough * normal
                    line = LineString([finite_vertex, far_point])

                else:
                    line = LineString([vor.vertices[v1], vor.vertices[v2]])
                lines.append(line)
            try:
                region_poly = Polygon(LineString(np.concatenate([l.coords for l in lines])).convex_hull)
            except Exception as e:
                print(e)
                pdb.set_trace()
                return None
        else:
            region_poly = Polygon([vor.vertices[i] for i in region])
        
        # Clip with boundry
        return region_poly.intersection(bounding_shape)

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

    voronoi_plugin = BoundedVoronoiPlugin(bounds=box(-9,-9, 9, 9))
    replayVisual.add_plugin(voronoi_plugin)

    replayVisual.start_animation()
    replayVisual.run()

if __name__ == '__main__':
    main()