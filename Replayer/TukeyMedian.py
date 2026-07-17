import numpy as np
import random
from shapely import Polygon,Point,MultiPoint,LineString

from itertools import combinations
from shapely.ops import unary_union



class TukeyContour:
    """
    Calculates the Tukey depth contour (median region) for a set of 2D points.

    """
    def __init__(self, input_points: np.ndarray, Xi: int, centerpoint: bool = False, mode: int = 1, verbose: bool = False):
        self.primal_points = np.asarray(input_points)
        self.verbose = verbose
        self.median_contour = np.array([])
        self.Xi = Xi
        self.mode = mode
        self.max_depth = None
        self.center_depth = None
        self.centerpoint = centerpoint
        if self.primal_points.shape[0] < 3:
            # Not enough points to form a contour
            return

        self._calculate_contour()
        
    def _cross_product(self, p1, p2, p3):
        """Calculates the 2D cross product to determine orientation."""
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    def _monotone_chain_convex_hull(self, points: np.ndarray):
        """Computes the convex hull of a set of 2D points."""
        points = sorted(points, key=lambda p: (p[0], p[1]))
        if len(points) <= 2:
            return points

        upper_hull, lower_hull = [], []
        for p in points:
            while len(lower_hull) >= 2 and self._cross_product(lower_hull[-2], lower_hull[-1], p) <= 0:
                lower_hull.pop()
            lower_hull.append(p)

        for p in reversed(points):
            while len(upper_hull) >= 2 and self._cross_product(upper_hull[-2], upper_hull[-1], p) <= 0:
                upper_hull.pop()
            upper_hull.append(p)

        return lower_hull[:-1] + upper_hull[:-1]

    def _calculate_contour(self):
        """
            Main logic to compute the Tukey median contour.
            mode 0: Self Distrust.
            mode 1: Normal.
            mode 2: Self Trust.
        """
        # 1. Duality Transform: Point (px, py) -> Line y = px*x - py
        # We store lines as (m, c) for y = mx + c
        dual_lines = np.array([[p[0], -p[1]] for p in self.primal_points])

        # 2. Find all intersection points of dual lines
        dual_intersections = []
        epsilon = 1e-9
        for i in range(len(dual_lines)):
            for j in range(i + 1, len(dual_lines)):
                if self.mode == 0:
                    # Exclude if either line belongs to the target_index
                    if i == self.Xi or j == self.Xi:
                        continue
                elif self.mode == 2:
                    # ONLY include if one of the lines belongs to target_index
                    if i != self.Xi and j != self.Xi:
                        continue
                m1, c1 = dual_lines[i]
                m2, c2 = dual_lines[j]
                if abs(m1 - m2) > epsilon:
                    x = (c2 - c1) / (m1 - m2)
                    y = m1 * x + c1
                    dual_intersections.append((x, y))

        # 3. Calculate the depth of each intersection point
        self.max_depth = 0
        intersections_with_depth = []
        for p in dual_intersections:
            px, py = p
            lines_above = np.sum((dual_lines[:, 0] * px + dual_lines[:, 1]) > py + epsilon)
            lines_below = np.sum((dual_lines[:, 0] * px + dual_lines[:, 1]) < py - epsilon)
            depth = min(lines_above, lines_below) + 1 # Depth is 1-indexed
            intersections_with_depth.append({'point': p, 'depth': depth})
            if depth > self.max_depth:
                self.max_depth = depth

        if self.verbose:
            print(f"Calculated depths. Maximum depth (k*) is {self.max_depth}.")

        # 4. Iteratively find a non-empty contour, starting from max_depth
        self.center_depth = np.ceil(len(self.primal_points)*1/3)
        final_contour_points = []
        if self.centerpoint:
            k = self.center_depth
        else:
            k = self.max_depth

        while k > 0 and not final_contour_points:
            median_dual_vertices = [item['point'] for item in intersections_with_depth if item['depth'] >= k]
            
            if len(median_dual_vertices) < 3:
                k -= 1
                continue

            # 5. Get the convex hull of the k-level region in dual space
            dual_contour_hull = self._monotone_chain_convex_hull(median_dual_vertices)

            # 6. Transform dual hull vertices back to primal lines
            primal_contour_lines = np.array([[p[0], -p[1]] for p in dual_contour_hull])

            # 7. Find intersections of these primal lines
            primal_vertices = []
            for i in range(len(primal_contour_lines)):
                for j in range(i + 1, len(primal_contour_lines)):
                    m1, c1 = primal_contour_lines[i]
                    m2, c2 = primal_contour_lines[j]
                    if abs(m1 - m2) > epsilon:
                        x = (c2 - c1) / (m1 - m2)
                        y = m1 * x + c1
                        primal_vertices.append((x, y))
            
            # 8. The final contour is the convex hull of these primal intersections
            if primal_vertices:
                final_contour_points = self._monotone_chain_convex_hull(primal_vertices)
            
            if not final_contour_points:
                if self.verbose:
                    print(f"Contour with depth {k} is empty, trying depth {k-1}")
                k -= 1

        self.median_contour = np.array(final_contour_points)

class SafePoint:
    def __init__(self):
        pass

    def CPIH_Safepoint(self, Bx, Xi, self_pos, mode=1):
        """
        CPIH-based resilient safe point.

        Parameters:
            Bx       : (n, m, 2) array of agent regions
            Xi       : index of this agent
            self_pos : np.array([x, y]) current robot position
            mode     : 0 (self distrust), 1 (normal), 2 (self trust)

        Returns:
            np.array([x, y]) safe point
        """

        n = Bx.shape[0]
        k = int(np.floor(2/3 * n) + 1)
        indices = np.arange(n)

        CPIH = Polygon()
        first = True

        for C in combinations(indices, k):

            # -----------------------------
            # Mode filtering
            # -----------------------------
            if mode == 0 and Xi in C:
                continue
            if mode == 2 and Xi not in C:
                continue

            Chull = Polygon()

            for triple in combinations(C, 3):
                b1, b2, b3 = map(int, triple)

                dp1_pts = np.vstack((Bx[b1], Bx[b2], Bx[b3]))
                verts = self._monotone_chain_convex_hull(dp1_pts)
                dp1 = Polygon(verts)

                dp2 = Polygon()
                for pair in combinations(triple, 2):
                    a1, a2 = pair
                    temp_pts = np.vstack((Bx[a1], Bx[a2]))
                    temp_poly = Polygon(temp_pts)
                    dp2 = unary_union([dp2, temp_poly])

                diff = dp1.difference(dp2)
                Chull = unary_union([Chull, diff])

            if Chull.is_empty:
                continue

            Chull = Polygon(Chull.convex_hull)

            # -----------------------------
            # Intersections
            # -----------------------------
            if first:
                CPIH = Chull
                first = False
            else:
                CPIH = CPIH.intersection(Chull)

                if not CPIH.is_empty and CPIH.geom_type != 'Polygon':
                    for geom in CPIH.geoms:
                        if geom.geom_type == 'Polygon':
                            CPIH = geom
                            break

            # Early exit
            if CPIH.is_empty:
                return np.array(self_pos)

        # -----------------------------
        # Final result
        # -----------------------------
        if CPIH.is_empty:
            return np.array(self_pos)

        centroid = CPIH.centroid
        return np.array([centroid.x, centroid.y])
    
    def CPIH_Fast_Safepoint(self, Bx, Xi, self_pos, mode=1):
        """
        Fast approximation of a Byzantine-safe point.

        Returns a robust centroid using distance-based filtering.
        Runs in O(n log n) instead of combinatorial time.
        """

        n = Bx.shape[0]

        # -----------------------------
        # Step 1: get representative point per agent
        # -----------------------------
        centers = np.mean(Bx, axis=1)   # shape (n,2)

        # -----------------------------
        # Step 2: mode filtering
        # -----------------------------
        if mode == 0:
            # distrust self
            pts = np.delete(centers, Xi, axis=0)

        elif mode == 2:
            # self-trust: keep nearest neighbors to self
            self_center = centers[Xi]

            dists = np.linalg.norm(centers - self_center, axis=1)

            k = int(np.floor(2/3 * n) + 1)

            sorted_idx = np.argsort(dists)

            pts = centers[sorted_idx[:k]]

        else:
            pts = centers

        if len(pts) == 0:
            return np.array(self_pos)

        # -----------------------------
        # Step 3: robust centroid
        # -----------------------------
        mean_pt = np.mean(pts, axis=0)

        dists = np.linalg.norm(pts - mean_pt, axis=1)

        k = int(np.floor(2/3 * len(pts)) + 1)

        safe_pts = pts[np.argsort(dists)[:k]]

        # -----------------------------
        # Step 4: keep closest k points
        # -----------------------------
        k = int(np.floor(2/3 * len(pts)) + 1)

        sorted_idx = np.argsort(dists)
        safe_pts = pts[sorted_idx[:k]]

        # -----------------------------
        # Step 5: return centroid
        # -----------------------------

        if len(safe_pts) == 1:
            # Single trusted point
            centroid = Point(safe_pts[0])
            region = centroid

        elif len(safe_pts) == 2:
            # Midpoint of segment
            midpoint = np.mean(safe_pts, axis=0)
            centroid = Point(midpoint)
            region = LineString(safe_pts)

        else:
            hull_pts = self._monotone_chain_convex_hull(safe_pts)
            region = Polygon(hull_pts)
            centroid = region.centroid

        tukey_depth = None
        if centroid is not None:
            tukey_depth = self.tukey_depth(
                np.array([centroid.x, centroid.y]),
                centers
            )
        centerpoint_depth = np.ceil(n/3)

        return centroid, region, tukey_depth, centerpoint_depth

    
    def _monotone_chain_convex_hull(self, points: np.ndarray):
        """Computes the convex hull of a set of 2D points."""
        points = sorted(points, key=lambda p: (p[0], p[1]))
        if len(points) <= 2:
            return points

        upper_hull, lower_hull = [], []
        for p in points:
            while len(lower_hull) >= 2 and self._cross_product(lower_hull[-2], lower_hull[-1], p) <= 0:
                lower_hull.pop()
            lower_hull.append(p)

        for p in reversed(points):
            while len(upper_hull) >= 2 and self._cross_product(upper_hull[-2], upper_hull[-1], p) <= 0:
                upper_hull.pop()
            upper_hull.append(p)

        return lower_hull[:-1] + upper_hull[:-1]

    def _cross_product(self, p1, p2, p3):
        """Calculates the 2D cross product to determine orientation."""
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    def tukey_depth(self, point, data):
        """
        Compute Tukey (halfspace) depth of a point.
        data: (n,2) array
        """
        n = len(data)
        min_halfspace = n

        for i in range(n):
            for j in range(i + 1, n):
                left = 0
                right = 0

                for k in range(n):
                    if k == i or k == j:
                        continue

                    cp = (
                        (data[j][0] - data[i][0]) * (data[k][1] - data[i][1])
                        - (data[j][1] - data[i][1]) * (data[k][0] - data[i][0])
                    )

                    if cp > 0:
                        left += 1
                    elif cp < 0:
                        right += 1

                min_halfspace = min(min_halfspace, left, right)

        return min_halfspace + 1

def clip_polygon(poly, a, b, c, d):
    new_poly = []
    if len(poly) == 0: return new_poly
    

    if d == 1:
        for j in range(len(poly)):
            p_curr = poly[j]
            p_next = poly[(j + 1) % len(poly)]
            
            val_curr = a * p_curr[0] + b * p_curr[1] + c
            val_next = a * p_next[0] + b * p_next[1] + c
            
            # If point is on the line, keep it
            if abs(val_curr) <= 1e-9:
                new_poly.append(p_curr)
                
            # If edge crosses the line  add intersection
            if (val_curr < -1e-9 and val_next > 1e-9) or \
                (val_curr > 1e-9 and val_next < -1e-9):
                denom = val_curr - val_next
                if abs(denom) > 1e-12:
                    t = val_curr / denom
                    inter_p = p_curr + t * (p_next - p_curr)
                    new_poly.append(inter_p)
        return np.array(new_poly)

    
    else:
        for j in range(len(poly)):
            p_curr = poly[j]
            p_next = poly[(j + 1) % len(poly)]
            
            val_curr = a * p_curr[0] + b * p_curr[1] + c
            val_next = a * p_next[0] + b * p_next[1] + c
            
            # Keep points inside
            if val_curr <= 1e-9:
                new_poly.append(p_curr)
            # Add intersection if crossing boundary
            if (val_curr <= 1e-9 and val_next > 1e-9) or (val_curr > 1e-9 and val_next <= 1e-9):
                denom = val_curr - val_next
                if abs(denom) > 1e-12:
                    t = val_curr / denom
                    inter_p = p_curr + t * (p_next - p_curr)
                    new_poly.append(inter_p)
        return np.array(new_poly)
        
def SelfTukeyMed(X, i, centerpoint=False):
    """
    X: nx2 matrix 
    i: index of z
    plot: boolean, plot == True: generates plot. plot == false: no plot, just returns depth and z-median vertices
 
    """
    z = X[i]
    n = len(X)
    centerpoint_depth = np.ceil(n/3)

    upper_klevels = []
    lower_klevels = []
    candidate_constraints = []
    
    indices = range(n)
    epsilon = 1e-9
    
    # Compute all k-levels
    for idx1, idx2 in combinations(indices, 2):
        p1 = X[idx1]
        p2 = X[idx2]
        dy = p2[1] - p1[1]
        dx = p2[0] - p1[0]
        a = -dy
        b = dx
        c = - (a * p1[0] + b * p1[1])
        
        # Check how many points above/below line
        vals = a * X[:, 0] + b * X[:, 1] + c
        
        n_above = np.sum(vals > epsilon)
        n_below = np.sum(vals < -epsilon)
        
        # Evaluate z position
        z_val = a * z[0] + b * z[1] + c
        z_is_above = z_val > epsilon
        z_is_below = z_val < -epsilon
        z_on_line  = abs(z_val) <= epsilon
        
        # Case 1: Less points above, keep lower halfspace
        if n_above < n_below:
            k = n_above
            if z_is_below or z_on_line:
                candidate_constraints.append((k, a, b, c, 0))
                if k == n/2-1:
                    upper_klevels.append(["num above: ", n_above, "p1: ", p1, " p2: ", p2])

        # Case 2: fewer points below, keep upper halfspace
        elif n_below < n_above:
            k = n_below
            if z_is_above or z_on_line:
                candidate_constraints.append((k, -a, -b, -c, 0))
                if k == n/2-1:
                    lower_klevels.append(["num below: ", n_below, "p1: ", p1, " p2: ", p2])

        # Case 3: Equal number above and below
        else:
            k = n_above
            if z_is_above:
                candidate_constraints.append((k, -a, -b, -c, 0))
            elif z_is_below:
                candidate_constraints.append((k, a, b, c, 0))
            else:
                # d=1: Intersect with line itself
                if (n%2 == 0):
                    candidate_constraints.append((k, a, b, c, 1)) 

    if not candidate_constraints:
        print("No valid constraints found.")
        return 0, np.array([]), [], []

    # find median depth
    if centerpoint:
        max_k = centerpoint_depth
    else:
        max_k = max(item[0] for item in candidate_constraints)
    median_found = False
    final_poly = np.array([]) 

    while (not median_found) and (max_k >= 0):
        active_constraints = [item for item in candidate_constraints if item[0] <= max_k]
        
        inf = 1e9
        poly = np.array([[-inf, -inf], [inf, -inf], [inf, inf], [-inf, inf]])
    
        # Apply constraints
        for _, a, b, c, d in active_constraints:
            if a == 0 and b == 0: continue
            poly = clip_polygon(poly, a, b, c, d)
            if len(poly) == 0: break 
        
        if len(poly) > 0:
            min_x, max_x = np.min(poly[:,0]), np.max(poly[:,0])
            min_y, max_y = np.min(poly[:,1]), np.max(poly[:,1])
            
            width = max_x - min_x
            height = max_y - min_y
            
            is_singular = (width < 1e-6) and (height < 1e-6)
            
            if not is_singular:
                median_found = True
                final_poly = poly
            else:
                max_k -= 1
        else:
            max_k -= 1

  
    
    return final_poly, max_k+1, centerpoint_depth