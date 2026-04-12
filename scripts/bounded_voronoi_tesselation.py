import warnings
from itertools import product
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import Polygon, LineString, Point, box
import matplotlib.pyplot as plt
import pdb

points = np.array([[-2.26700234, -5.15908527], [-2.05122352, -0.6547026 ], [-0.62305647,  1.71807921], [ 1.48229587,  0.59669858], [ 1.06217539, -2.12913156]])
# points = np.array([[0, 0], [1, 0], [0, 1]])
track_pt = np.array([0.25, 0.75])
track_wt = np.array([0.5,0.5]) #dynamically set later

vor = Voronoi(points)

# for (point_idx1, point_idx2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
#     if v2 == -1 or v1 == -1:
#         # Get the two original points that generated this ridge
#         p1 = vor.points[point_idx1]
#         p2 = vor.points[point_idx2]

#         # Compute midpoint of p1 and p2 (for visualizing or anchoring)
#         midpoint = (p1 + p2) / 2

#         # Compute the vector between them
#         line_vec = p2 - p1

#         # Compute the normal (perpendicular vector)
#         direction = np.array([-line_vec[1], line_vec[0]])
#         direction = direction / np.linalg.norm(direction)  # Normalize

#         # Choose Direction
#         finite_vertex = vor.vertices[v1 if v1 != -1 else v2]
#         to_vertex = finite_vertex - midpoint

#         # If dot product is negative, we are going the wrong way
#         if np.dot(to_vertex, direction) > 0:
#             direction *= -1 

#         print(f"Ridge from point {point_idx1} to {point_idx2}")
#         print(f"Midpoint: {midpoint}")
#         print(f"Direction to infinity: {direction}")

# fig, ax = plt.subplots()
# voronoi_plot_2d(vor, ax=ax, show_vertices=True, line_colors='black', point_size=10)
# ax.set_xlim(-0.25, 1.25)
# ax.set_ylim(-0.25, 1.25)
# ax.set_aspect('equal')
# ax.set_title("Ubounded")
# plt.tight_layout()
# plt.show()


'''
vor.points          # original input points
vor.vertices        # vertices of the voroni intersections
vor.regions         # List of regions, each list of indices into vor.vertices
vor.point_region    # For each point in vor.points, give the index of the region in vor.regions that it belongs
vor.ridge_points    # Array of shape (n_ridges, 2) with indices of the two input points that share a Voronoi Ridge
vor.ridge_vertices  # List of index pairs from vor.vertices forming the edges between regions
vor.ridge_dict      # A dictionary mapping a tuple of sorted point indices to the indices of the vertices forming the ridge
vor.furthest_site   # Only if furthest_site=True; Boolean flag indicating that the diagram was computed as a furthest-site Voronoi diagram
'''

# Define bounding box (xmin, ymin, xmax, ymax)
bounding_rect = box(-0.25, -0.25, 1.25, 1.25)
bounding_circle = Point((0.5, 0.5)).buffer(0.7, resolution=100)
bounding_general = Polygon([(-0.1,-0.1), (-0.25,1), (1, 1.25), (1.25, 0)])
bounding_shape_general = Polygon([(-3.8, 0.19), (-0.12, 3.36), (2.69, 2.0), (2.74, -2.6), (-0.18, -5.0), (-2.36, -5.12)])

def get_points(vor, v1, v2, bounding_shape, far_enough=100):
    if v1 == -1:
        point_idx = v2
    else:
        point_idx = v1

    point = Point(vor.vertices[point_idx])
    closest_point = bounding_shape.exterior.interpolate(
        bounding_shape.exterior.project(point)
    )
    
    start_pt = vor.vertices[point_idx]
    direction_pt = np.array([closest_point.x, closest_point.y])
    vector = direction_pt - start_pt
    vector /= np.linalg.norm(vector)

    end_pt = start_pt + far_enough * vector

    return start_pt, end_pt


# Reconstruct finite Voronoi polygons with clipping
def bounded_voronoi_region1(vor, point_idx, bounding_shape, far_enough=100):
    region_idx = vor.point_region[point_idx]
    region = vor.regions[region_idx]
    if -1 in region or len(region) == 0:
        # Region is infinite, reconstruct polygon with ridge directions
        point = vor.points[point_idx]
        ridges = [r for r in vor.ridge_points if point_idx in r]
        lines = []
        for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
            if point_idx not in (p1, p2):
                continue
            if v1 == -1 or v2 == -1:
                if v1 == -1:
                    point_index = v2
                else:
                    point_index = v1
                
                midpoint = vor.vertices[point_index]
                point_special = Point(vor.vertices[point_index])

                # Check to see if my midpoint is inside the boundary
                if not bounding_shape.boundary.contains(point_special):
                # if True:
                    closest_point = bounding_shape.exterior.interpolate(
                        bounding_shape.exterior.project(point_special)
                    )
                    
                    direction_pt = np.array([closest_point.x, closest_point.y])
                    normal = direction_pt - midpoint
                    normal /= np.linalg.norm(normal)

                    if not bounding_shape.contains(point_special):
                        normal *= -1    
                else:
                    # need to find normal out of boundary
                    coords = np.array(bounding_shape.exterior.coords)
                    
                    min_dist = float('inf')
                    seg_index = None
                    for i in range(len(coords)-1):
                        seg = LineString([coords[i], coords[i+1]])
                        d = seg.distance(point_special)
                        if d < min_dist:
                            min_dist = d 
                            seg_index = i
                    
                    p1 = coords[seg_index]
                    p2 = coords[seg_index+1]
                    tangent = (p2 - p1) / np.linalg.norm(p2 - p1)

                    normal = np.array([-tangent[1], tangent[0]])

                    if bounding_shape.contains(Point(midpoint + normal)):
                        normal *= -1 

                far_point = midpoint + far_enough * normal
                line = LineString([midpoint, far_point])

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

# Reconstruct finite Voronoi polygons with clipping
def bounded_voronoi_region(vor, point_idx, bounding_shape, far_enough=100):
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

# Get centroids for all regions
centroids1 = None
centroids = []
region_polys = []
for i in range(len(points)):
    # # if I only care about one point
    # if i != 1:
    #     continue
    
    # region_poly = bounded_voronoi_region(vor, i, bounding_circle)
    # region_poly = bounded_voronoi_region(vor, i, bounding_rect)
    # region_poly = bounded_voronoi_region(vor, i, bounding_general)
    region_poly = bounded_voronoi_region(vor, i, bounding_shape_general)

    if type(region_poly) != type(None):
        x,y = region_poly.exterior.xy

        if region_poly and not region_poly.is_empty:
            centroid = region_poly.centroid
            if type(centroids1) == type(None):
                centroids1 = np.array([(centroid.x, centroid.y)])
            else:
                centroids1 = np.vstack([centroids1, (centroid.x, centroid.y)])
            centroids.append((centroid.x, centroid.y))
        else:
            centroids1 = np.append(centroids1, (None))
            centroid.append((None))
        region_polys.append(region_poly)
    else:
        print(f"Point {i} didn't have a region")

# Add Tracking to a point
# The closer you are to the tracker, the more you move toward it. 
def compute_auto_weights(new_coordinates):
    global points
    dists = np.linalg.norm(new_coordinates - points, axis=1)
    min_ = dists.min()
    max_ = dists.max()
    dists_norm = (dists - min_) / (max_ - min_) if max_ != min_ else np.zeros(dists.shape)

    return 1 - dists_norm

track_pt_single = track_pt
# track_wt = compute_auto_weights(track_pt)
# track_pt = np.tile(track_pt, [centroids1.shape[0], 1])
# track_wt = track_wt.reshape(-1, 1)


midpoints = (centroids + track_pt * track_wt) / (1 + track_wt)

# Plotting
fig, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(12, 6))
ax1.plot(points[:,0], points[:,1], 'ko', label='Input Points')
ax2.plot(points[:,0], points[:,1], 'ko', label='Input Points')
ax2.plot(track_pt_single[0], track_pt_single[1], 'bo', label='Tracking Point')

for region_poly, centroid, midpoint in zip(region_polys, centroids, midpoints):
    if region_poly and not region_poly.is_empty:
        x, y = region_poly.exterior.xy
        ax1.fill(x, y, alpha=0.2, edgecolor='black')
        ax2.fill(x, y, alpha=0.2, edgecolor='black')
        if centroid:
            ax1.plot(*centroid, 'r+', markersize=10)

            ax2.plot(*centroid, 'g+', markersize=7)
            ax2.plot([centroid[0], midpoint[0]], [centroid[1], midpoint[1]], color='red')
            ax2.plot(*midpoint, 'r+', markersize=10)

# xmin, xmax = -0.25, 1.25
# ymin, ymax = -0.25, 1.25
xmin, xmax = -6, 5
ymin, ymax = -6, 5

ax1.set_xlim(xmin, xmax)
ax1.set_ylim(ymin, ymax)
ax1.set_aspect('equal')
ax1.set_title("Bounded Voronoi Diagram with Region Centroids")
ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5))

ax2.set_xlim(xmin, xmax)
ax2.set_ylim(ymin, ymax)
ax2.set_aspect('equal')
ax2.set_title("Bounded Voronoi Diagram with Weighted Tracking")
ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))



# point = vor.points[5]
# ridges = [r for r in vor.ridge_points if 5 in r]

# print(point)
# print(ridges)
# print(vor.ridge_points)
# print(vor.ridge_vertices)

vor2 = Voronoi(midpoints)
voronoi_plot_2d(vor2, ax=ax3, show_vertices=True, line_colors='black', point_size=10)
ax3.set_xlim(xmin, xmax)
ax3.set_ylim(ymin, ymax)
ax3.set_aspect('equal')
ax3.set_title("New Weighted")
voronoi_plot_2d(vor, ax=ax4, show_vertices=True, line_colors='black', point_size=10)
ax4.set_xlim(xmin, xmax)
ax4.set_ylim(ymin, ymax)
ax4.set_aspect('equal')
ax4.set_title("Ubounded")
plt.tight_layout()
plt.show()
