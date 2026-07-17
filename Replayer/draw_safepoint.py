import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import Polygon, Point, LineString

from TukeyMedian import TukeyContour, SafePoint, SelfTukeyMed


def plot_polygon(poly, ax, **kwargs):
    """Draw numpy polygon or shapely polygon."""

    if poly is None:
        return

    if isinstance(poly, Polygon):
        if poly.is_empty:
            return
        x, y = poly.exterior.xy
        ax.fill(x, y, **kwargs)
        return

    poly = np.asarray(poly)

    if len(poly) == 0:
        return

    poly = np.vstack((poly, poly[0]))
    ax.fill(poly[:,0], poly[:,1], **kwargs)


def plot_imprecision_regions(Bx, ax):

    for i, region in enumerate(Bx):

        region = np.asarray(region)

        region = np.vstack((region, region[0]))

        ax.plot(region[:,0], region[:,1],
                color="gray",
                linewidth=1)

        ax.fill(region[:,0],
                region[:,1],
                color="gray",
                alpha=.15)


def plot_safepoint(X, Bx,
                   Xi=0,
                   centerpoint=True,
                   mode=1):

    from scipy.spatial import ConvexHull

    #############################################################
    # Compute everything once
    #############################################################

    tc = TukeyContour(
        X,
        Xi,
        centerpoint=centerpoint,
        mode=mode
    )

    final_poly, tukey_depth, center_depth = SelfTukeyMed(
        X,
        Xi,
        centerpoint
    )

    sp = SafePoint()

    centroid, region, depth, cpdepth = sp.CPIH_Fast_Safepoint(
        Bx,
        Xi,
        X[Xi],
        mode
    )

    #############################################################
    # Figure layout
    #############################################################

    fig, axs = plt.subplots(2, 2, figsize=(14, 12))

    ax_geom  = axs[0, 0]
    ax_tukey = axs[0, 1]
    ax_safe  = axs[1, 0]
    ax_final = axs[1, 1]

    #############################################################
    # Helper Functions
    #############################################################

    def draw_points(ax):

        ax.scatter(
            X[1:, 0],
            X[1:, 1],
            s=70,
            color="dodgerblue",
            label="Neighbors",
            zorder=5
        )

        ax.scatter(
            X[Xi, 0],
            X[Xi, 1],
            s=120,
            color="red",
            edgecolor="black",
            label="Self",
            zorder=6
        )

        for i, p in enumerate(X):
            ax.text(p[0], p[1], str(i), fontsize=10)

    def draw_convex_hull(ax):

        if len(X) < 3:
            return

        hull = ConvexHull(X)

        hull_pts = X[hull.vertices]
        hull_pts = np.vstack((hull_pts, hull_pts[0]))

        ax.plot(
            hull_pts[:,0],
            hull_pts[:,1],
            '--',
            color='black',
            linewidth=1,
            label='Convex Hull'
        )

    def draw_tukey(ax):

        if len(tc.median_contour):

            plot_polygon(
                tc.median_contour,
                ax,
                color='green',
                alpha=.25,
                label='Tukey Contour'
            )

        if len(final_poly):

            plot_polygon(
                final_poly,
                ax,
                color='orange',
                alpha=.30,
                label='Self Tukey Median'
            )

    def draw_safe_region(ax):

        if isinstance(region, Polygon):

            x, y = region.exterior.xy

            ax.fill(
                x,
                y,
                color='purple',
                alpha=.25,
                label='Safe Region'
            )

        elif isinstance(region, LineString):

            x, y = region.xy

            ax.plot(
                x,
                y,
                color='purple',
                linewidth=3,
                label='Safe Region'
            )

        if centroid is not None:

            ax.scatter(
                centroid.x,
                centroid.y,
                s=180,
                marker='*',
                color='gold',
                edgecolor='black',
                label='SafePoint',
                zorder=10
            )

    #############################################################
    # Geometry subplot
    #############################################################

    plot_imprecision_regions(Bx, ax_geom)
    draw_points(ax_geom)
    draw_convex_hull(ax_geom)

    ax_geom.set_title("Input Geometry")

    #############################################################
    # Tukey subplot
    #############################################################

    draw_points(ax_tukey)
    draw_tukey(ax_tukey)

    ax_tukey.set_title(
        f"Tukey Analysis\nDepth={tukey_depth}"
    )

    #############################################################
    # SafePoint subplot
    #############################################################

    plot_imprecision_regions(Bx, ax_safe)
    draw_points(ax_safe)
    draw_safe_region(ax_safe)

    ax_safe.set_title("SafePoint")

    #############################################################
    # Combined subplot
    #############################################################

    plot_imprecision_regions(Bx, ax_final)
    draw_points(ax_final)
    draw_convex_hull(ax_final)
    draw_tukey(ax_final)
    draw_safe_region(ax_final)

    ax_final.set_title("Combined")

    #############################################################
    # Formatting
    #############################################################

    info = (
        f"Tukey Depth: {tukey_depth}\n"
        f"Center Depth: {center_depth}\n"
        f"Mode: {mode}\n"
        f"Robots: {len(X)}"
    )

    for ax in axs.flat:

        ax.set_aspect("equal")
        ax.grid(True)
        ax.legend(loc="best")

        ax.text(
            0.02,
            0.98,
            info,
            transform=ax.transAxes,
            va='top',
            bbox=dict(facecolor='white', alpha=.85)
        )

    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.01)

# if __name__ == '__main__':
#     X = np.zeros((len(viz.data.neighbor_position[frame])+1,2))

#     X[0] = [viz.data.x_vals[frame], viz.data.y_vals[frame]]

#     i = 1
#     for _, neighbor in viz.data.neighbor_position[frame].items():
#         X[i] = neighbor
#         i += 1

#     Bx = self.getImprecisionRegions(
#         X,
#         viz.data.imprecision[frame]
#     )

#     plot_safepoint(
#         X,
#         Bx,
#         Xi=0,
#         centerpoint=True,
#         mode=viz.data.self_trust[frame]
#     )