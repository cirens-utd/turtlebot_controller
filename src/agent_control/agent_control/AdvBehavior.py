#!/usr/bin/env python3
import rclpy
from rclpy.parameter import Parameter
from agent_control.agent import Agent
import numpy as np
import traceback
import pdb
import numpy as np
from shapely import Polygon,Point,MultiPoint,LineString
from shapely.ops import polygonize, unary_union
from itertools import combinations

EPS = 1e-9


class Line:
    def __init__(self,a= 1,b=1,c=1,p1 = [], p2 = [], sign = 0):
        norm = np.sqrt(a*a+b*b)
        self.A = a/norm
        self.B = b/norm
        self.C = c/norm
        self.sign = sign
        self.p1 = -1
        self.p2 = -1
        if (len(p1)>0 and len(p2)>0):
            A,B,C = self.line_coeffs(p1,p2)
            self.A = A
            self.B = B
            self.C = C
            self.p1 = p1
            self.p2 = p2
        
    def side(self, p, eps = 1e-10):
        s = self.A*p[0]+self.B*p[1]+self.C
        if abs(s) < eps:
            return 0
        return np.sign(s)
    
    def line_coeffs(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        A = y2 - y1
        B = x1 - x2
        C = x2*y1 - x1*y2
        return A, B, C


class Wedge:
    def __init__(self, L1, L2, X=None):
        self.L1 = L1
        self.L2 = L2
        self.sign1 = L1.sign
        self.sign2 = L2.sign
        self.apex = line_intersection(L1,L2)
        self.bisector_direction = self.bisecting_direction()

        # Robot classification
        self.robots_inside = []
        self.robots_outside = []
        self.robots_online = []

        if X is not None:
            self.classify_robots_(X)

    def classify_robots_(self, X, eps=1e-10):

        self.robots_inside = []
        self.robots_outside = []
        self.robots_online = []

        for i, point in enumerate(X):

            s1 = (
                self.L1.A * point[0]
                + self.L1.B * point[1]
                + self.L1.C
            )

            s2 = (
                self.L2.A * point[0]
                + self.L2.B * point[1]
                + self.L2.C
            )

            # Robot lies on either boundary
            if abs(s1) < eps or abs(s2) < eps:
                self.robots_online.append(i)

            # Robot lies strictly inside selected wedge
            elif (
                self.L1.sign * s1 > eps
                and
                self.L2.sign * s2 > eps
            ):
                self.robots_inside.append(i)

            # Otherwise it is outside
            else:
                self.robots_outside.append(i)
    
    def contains(self,p):
        pos = self.L1.sign*(self.L1.A*p[0]+self.L1.B*p[1]+self.L1.C)>0 and self.L2.sign*(self.L2.A*p[0]+self.L2.B*p[1]+self.L2.C)>0
        neg = -self.L1.sign*(self.L1.A*p[0]+self.L1.B*p[1]+self.L1.C)>0 and -self.L2.sign*(self.L2.A*p[0]+self.L2.B*p[1]+self.L2.C)>0
        return pos or neg
    
    def bisecting_direction(self):
        """
        Returns a unit vector pointing from the apex into the wedge.
        """

        v1 = np.array([
            self.sign1 * self.L1.A,
            self.sign1 * self.L1.B
        ], dtype=float)

        v2 = np.array([
            self.sign2 * self.L2.A,
            self.sign2 * self.L2.B
        ], dtype=float)

        v = v1 + v2
        norm = np.linalg.norm(v)

        if norm < EPS:
            return None

        return v / norm

    def get_hull_lines(self):
        L1p1 = self.L1.p1
        L1p2 = self.L1.p2
        L2p1 = self.L2.p1
        L2p2 = self.L2.p2
        points = [L1p1, L1p2,L2p1,L2p2]
        v1= np.array([self.sign1*self.L1.A, self.sign1*self.L1.B])
        v2 = np.array([self.sign2*self.L2.A, self.sign2*self.L2.B])
        v = v1+v2
        p = self.apex
        vals = []
        for point in points:
            u = point-p
            t = np.dot(u,v)/np.dot(v,v)
            vals.append(t)
        indices = np.argsort(vals)
        hull1 = Line(p1 = points[indices[0]], p2 = points[indices[1]])
        hull2 = Line(p1 = points[indices[2]], p2 = points[indices[3]])
        hull1.sign = -(np.sign(hull1.A*self.apex[0]+hull1.B*self.apex[1]+hull1.C))
        hull2.sign = -hull1.sign
        return hull1,hull2

def clip_line_to_box(A, B, C, BIG=[-10.0,10,-10,10]):
    pts = []
    for x in (BIG[0], BIG[1]):
        if abs(B) > 1e-12:
            y = -(A*x + C)/B
            if BIG[2]-1e-9 <= y <= BIG[3]+1e-9: pts.append((x, y))
    for y in (BIG[2], BIG[3]):
        if abs(A) > 1e-12:
            x = -(B*y + C)/A
            if BIG[0]-1e-9 <= x <= BIG[1]+1e-9: pts.append((x, y))
    uniq = []
    for q in pts:
        if not any(abs(q[0]-r[0])<1e-7 and abs(q[1]-r[1])<1e-7 for r in uniq):
            uniq.append(q)
    return LineString(uniq[:2]) if len(uniq) >= 2 else None

def bisector_target(wedge, BIG=[-10.0,10,-10,10], distance=None):

    apex = np.asarray(wedge.apex, dtype=float)
    direction = wedge.bisector_direction

    if direction is None:
        return None

    # User explicitly requested a distance.
    if distance is not None:
        return apex + distance * direction

    # Otherwise find where the ray hits the BIG box.
    dx, dy = direction
    x0, y0 = apex

    candidates = []

    if abs(dx) > EPS:
        t = (BIG[1] - x0) / dx
        if t > EPS:
            y = y0 + t * dy
            if BIG[2] - EPS <= y <= BIG[3] + EPS:
                candidates.append(t)

    if abs(dx) > EPS:
        t = (BIG[0] - x0) / dx
        if t > EPS:
            y = y0 + t * dy
            if BIG[2] - EPS <= y <= BIG[3] + EPS:
                candidates.append(t)

    if abs(dy) > EPS:
        t = (BIG[3] - y0) / dy
        if t > EPS:
            x = x0 + t * dx
            if BIG[0] - EPS <= x <= BIG[1] + EPS:
                candidates.append(t)

    if abs(dy) > EPS:
        t = (BIG[2] - y0) / dy
        if t > EPS:
            x = x0 + t * dx
            if BIG[0] - EPS <= x <= BIG[1] + EPS:
                candidates.append(t)

    if not candidates:
        return None

    # Smallest positive t = first boundary encountered
    t = min(candidates)

    return apex + t * direction

def face_contains_point(face, p):
    """
    Returns True if point p is inside or on the boundary of the face.
    """
    return face.covers(Point(p))

def face_bisector_target(face, center, BIG=[-10.0,10,-10,10], distance=None):
    """
    Treat the direction from `center` to the center of the face as
    the face's bisector direction.

    If distance is None:
        return the point where the ray hits the BIG box.

    Otherwise:
        return center + distance * direction.
    """

    center = np.asarray(center, dtype=float)

    # representative_point() is guaranteed to be inside the polygon.
    face_point = face.representative_point()
    face_center = np.array([
        face_point.x,
        face_point.y
    ], dtype=float)

    direction = face_center - center
    norm = np.linalg.norm(direction)

    if norm < EPS:
        return None

    direction /= norm

    # User explicitly requested a distance.
    if distance is not None:
        return center + distance * direction

    # Otherwise find where the ray hits the BIG box.
    dx, dy = direction
    x0, y0 = center

    candidates = []

    if abs(dx) > EPS:
        t = (BIG[1] - x0) / dx
        if t > EPS:
            y = y0 + t * dy
            if BIG[2] - EPS <= y <= BIG[3] + EPS:
                candidates.append(t)

        t = (BIG[0] - x0) / dx
        if t > EPS:
            y = y0 + t * dy
            if BIG[2] - EPS <= y <= BIG[3] + EPS:
                candidates.append(t)

    if abs(dy) > EPS:
        t = (BIG[3] - y0) / dy
        if t > EPS:
            x = x0 + t * dx
            if BIG[0] - EPS <= x <= BIG[1] + EPS:
                candidates.append(t)

        t = (BIG[2] - y0) / dy
        if t > EPS:
            x = x0 + t * dx
            if BIG[0] - EPS <= x <= BIG[1] + EPS:
                candidates.append(t)

    if not candidates:
        return None

    t = min(candidates)

    return center + t * direction

def colorful_selection(p, wedge_sets):
    chosen = []
    for i, wedges in enumerate(wedge_sets):
        for j, w in enumerate(wedges):
            if w.contains(p):      
                chosen.append((i, j, w))
                break               
    return chosen

def max_color_selection(wedge_sets, X, Y, previous_target=None, BIG=[-10.0,10,-10,10], alpha=3.0, beta=8.0, gamma=100.0, bisector_distance=None):
    # alpha = importance of staying near most neighbors
    # beta  = importance of not changing targets
    # gamma = importance of avoiding the hull

    segments = []

    for wedges in wedge_sets:
        for w in wedges:
            for L in (w.L1, w.L2):
                seg = clip_line_to_box(L.A, L.B, L.C, BIG)
                if seg is not None:
                    segments.append(seg)

    xmin, xmax, ymin, ymax = BIG

    box = Polygon([
        (xmin, ymin),
        (xmax, ymin),
        (xmax, ymax),
        (xmin, ymax)
    ])

    hull = Polygon(X).convex_hull

    # segments.append(hull.exterior)
    segments.append(box.exterior)

    faces = list(polygonize(unary_union(segments)))

    if len(faces) == 0:
        return None

    hull_centroid = np.array([
        hull.centroid.x,
        hull.centroid.y
    ], dtype=float)

    face_candidates = []

    for face in faces:

        if face.is_empty or not face.touches(box.boundary):
            continue

        representative = face.representative_point()

        face_point = np.array([
            representative.x,
            representative.y
        ], dtype=float)

        selection = colorful_selection(
            face_point,
            wedge_sets
        )

        if len(selection) == 0:
            continue

        target = face_bisector_target(
            face,
            hull_centroid,
            BIG=BIG,
            distance=bisector_distance
        )

        if target is None:
            continue

        target = np.asarray(target, dtype=float)

        # Don't allow target inside formation hull.
        if hull.contains(Point(target)):
            continue

        target_selection = colorful_selection(
            (target[0], target[1]),
            wedge_sets
        )

        k = len(target_selection)

        if k == 0:
            continue

        # Penalize for being farther away from starting position
        dists = sorted(np.linalg.norm(target - y) for y in Y)

        majority = len(Y) // 2 + 1
        majority = min(majority, len(dists))

        if majority > 0:
            majority_distance = sum(dists[:majority])
        else:
            majority_distance = 0.0

        # Penalize changeing zones
        if previous_target is None:
            jump = 0.0
        else:
            jump = np.linalg.norm(target - np.asarray(previous_target))

        center_distance = np.linalg.norm(target - hull_centroid)

        # Penalize going through the hull of the formation
        hull_interstection_distance = 0
        for y in Y:
            path = LineString([
                tuple(y),
                tuple(target)
            ])

            intersection = path.intersection(hull)

            if intersection.is_empty:
                hull_crossing_distance = 0.0
            else:
                hull_crossing_distance = intersection.length
            hull_interstection_distance += hull_crossing_distance

        score = (
            10 * k 
            - alpha * majority_distance
            - beta * jump
            - gamma * hull_interstection_distance
        )

        face_candidates.append({
            "face": face,
            "face_point": face_point,
            "selection": selection,
            "target": target,
            "target_selection": target_selection,
            "k": k,
            "majority_distance": majority_distance,
            "jump": jump,
            "center_distance": center_distance,
            "score": score
        })

    if len(face_candidates) == 0:
        return None

    selected_face = max(
        face_candidates,
        key=lambda candidate: candidate["score"]
    )

    return (
        selected_face["k"],
        tuple(selected_face["target"]),
        selected_face["target_selection"],
        selected_face["score"],
        hull,
        face_candidates
    )

def dist_to_line(pt, line):
    return abs(line.A*pt[0] + line.B*pt[1] + line.C)

def line_intersection(L1, L2):
    A = np.array([[L1.A, L1.B],
                [L2.A, L2.B]])
    b = -np.array([L1.C, L2.C])
    if abs(np.linalg.det(A)) < 1e-10:
        return None

    return np.linalg.solve(A,b)

def line_coeffs(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    A = y2 - y1
    B = x1 - x2
    C = x2*y1 - x1*y2
    return A, B, C

def side(A, B, C, p, eps=1e-10):
    s = A*p[0] + B*p[1] + C
    if abs(s) < eps:
        return 0
    return np.sign(s)

def get_projected_pos( Y, target_line, hull_lines =[], buffer = 2.0):
    '''
    --Returns candidate target positions for Y--

    target_line: Directed line obtained from angle_bisectors().  
    hull_lines: obtained from get_boundary_lines()
    Step 1: projects the points of Y orthogonally onto target_line
    Step 2: computes first_point which is the first projected point of Y in the direction of target_line
    Step 3: checks if any projected point is inside the convex hull of X.  If so, the distance along 
    target_line from first_point to the intersection point of target_line with the convex hull of X is found
    and all projected points are moved along the direction of target_line by that distance (plus buffer)
      
    '''
    v = np.array([-target_line.A,target_line.B])
    vdir = v/np.linalg.norm(v)
    p = np.array([0, -target_line.C/target_line.B])
    proj_positions = []
    m = len(Y)
    outside_hull = np.zeros(m)
    intersections = []
    first_val = 10000
    first_point = []
    clean = False
    for y in Y:
        u = y-p
        t = np.dot(u,v)/np.dot(v,v)
        if t<first_val:
            first_val = t
            first_point = y
        proj_positions.append(p+t*v)

    for line in hull_lines:
        intersect = line_intersection(target_line, line)       
        i = 0
        for pos in proj_positions:
            dist = np.linalg.norm(intersect-pos)
            intersections.append([intersect, dist])
            if  line.sign*(line.A*pos[0]+line.B*pos[1]+line.C)>0:
                #print("pos: ", pos[0],",",pos[1], " is on the right side of A: ", line.A," B: ", line.B, " C: ", line.C)
                outside_hull[i] = 1
            i+=1
    if  sum(outside_hull) < m:
        intersections = sorted(intersections, key=lambda x: x[1])
        for i in range(len(intersections)):
            t_v = (intersections[i][0][0]-first_point[0])/vdir[0]
            dist = intersections[i][1]
            if t_v>0:
                clean = True
                for i in range(len(proj_positions)):
                    proj_positions[i] = proj_positions[i]+(dist+buffer)*vdir
                break
    else: 
        clean = True
    if clean:
        return proj_positions
    else: 
        return []

def line_balance_info(L, X, eps=1e-10):
    """
    Analyze how line L partitions the robots.

    Returns:
        on_line  = number of robots lying on the line
        positive = number of robots strictly on the + side
        negative = number of robots strictly on the - side
        balanced = whether the two sides differ by at most 1
    """

    on_line = 0
    positive = 0
    negative = 0

    for point in X:
        s = L.A * point[0] + L.B * point[1] + L.C

        if abs(s) < eps:
            on_line += 1

        elif s > 0:
            positive += 1

        else:
            negative += 1

    balanced = abs(positive - negative) <= 1

    return on_line, positive, negative, balanced

def is_almost_balanced_line(L, X):
    on_line, positive, negative, balanced = line_balance_info(L, X)

    return balanced

def get_unique_candidate_lines(X, eps=1e-10):
    """
    Generate unique geometric lines determined by pairs of robots.

    Multiple robot pairs may lie on the same geometric line.
    This function collapses those duplicates.
    """

    n = len(X)
    pairs = list(combinations(range(n), 2))

    unique_lines = []

    for i, j in pairs:

        A, B, C = line_coeffs(X[i], X[j])

        L = Line(
            A,
            B,
            C,
            p1=X[i],
            p2=X[j]
        )

        # ------------------------------------------
        # Check whether this geometric line is
        # already represented.
        # ------------------------------------------

        duplicate = False

        for existing in unique_lines:

            # Because Line normalizes A and B, compare
            # normalized coefficients.
            if (
                abs(L.A - existing.A) < eps
                and
                abs(L.B - existing.B) < eps
                and
                abs(L.C - existing.C) < eps
            ):
                duplicate = True
                break

            # Same line can also have all coefficients
            # negated depending on point ordering.
            if (
                abs(L.A + existing.A) < eps
                and
                abs(L.B + existing.B) < eps
                and
                abs(L.C + existing.C) < eps
            ):
                duplicate = True
                break

        if not duplicate:
            unique_lines.append(L)

    return unique_lines

def wedge_contains_robot(L1, L2, point, eps=1e-10):
    """
    Return True if point lies strictly inside the wedge
    defined by the positive sides of L1 and L2.

    Points lying exactly on either boundary are not considered
    to be inside the wedge.
    """

    s1 = L1.A * point[0] + L1.B * point[1] + L1.C
    s2 = L2.A * point[0] + L2.B * point[1] + L2.C

    return (L1.sign * s1 > eps) and (L2.sign * s2 > eps)

def wedge_is_empty(L1, L2, X, eps=1e-10):
    """
    Return True if no robot lies strictly inside the wedge.
    """

    for point in X:
        if wedge_contains_robot(L1, L2, point, eps):
            return False

    return True

def get_boundary_lines(X):
    """
    Find pairs of approximately balanced boundary lines.

    A valid boundary line:
        - passes through at least 2 robots
        - may pass through 3, 4, ... robots
        - ignores robots lying on the line
        - divides the remaining robots as evenly as possible

    Therefore:
        even number off-line -> exact split
        odd number off-line  -> difference of exactly 1 allowed
        Wedges will be in pairs, Both side need to be empty to be valid
    """

    # ==========================================
    # Generate unique geometric lines
    # ==========================================

    candidate_lines = get_unique_candidate_lines(X)

    # ==========================================
    # Keep only balanced lines
    # ==========================================

    balanced_lines = []

    for L in candidate_lines:

        on_line, positive, negative, balanced = \
            line_balance_info(L, X)

        if not balanced:
            continue

        # A valid boundary line must contain
        # at least two robots.
        if on_line < 2:
            continue

        L.on_line_count = on_line
        L.positive_count = positive
        L.negative_count = negative

        balanced_lines.append(L)

    # ==========================================
    # Create wedges from pairs of valid lines
    # ==========================================

    boundary_wedges = []

    for i in range(len(balanced_lines)):

        L1 = balanced_lines[i]

        for j in range(i + 1, len(balanced_lines)):

            L2 = balanced_lines[j]

            # --------------------------------------
            # Don't use identical geometric lines
            # --------------------------------------

            if (
                abs(L1.A - L2.A) < EPS
                and
                abs(L1.B - L2.B) < EPS
                and
                abs(L1.C - L2.C) < EPS
            ):
                continue

            # --------------------------------------
            # Don't use parallel lines
            # --------------------------------------

            D = L1.A * L2.B - L1.B * L2.A

            if abs(D) < EPS:
                continue

            # ======================================
            # Try all four sign combinations
            # ======================================

            sign_combinations = [
                ((1, 1), (-1, -1)),
                ((1, -1), (-1, 1),)
            ]

            for (sign1, sign2), (opp_sign1, opp_sign2) in sign_combinations:

                test_L1 = Line(
                    L1.A,
                    L1.B,
                    L1.C,
                    sign=sign1
                )

                test_L2 = Line(
                    L2.A,
                    L2.B,
                    L2.C,
                    sign=sign2
                )

                opposite_L1 = Line(
                    L1.A,
                    L1.B,
                    L1.C,
                    sign=opp_sign1
                )

                opposite_L2 = Line(
                    L2.A,
                    L2.B,
                    L2.C,
                    sign=opp_sign2
                )

                if not wedge_is_empty(test_L1, test_L2, X):
                    continue

                if not wedge_is_empty(opposite_L1, opposite_L2, X):
                    continue

                boundary_wedges.append(
                    Wedge(
                        test_L1,
                        test_L2,
                        X
                    )
                )

                boundary_wedges.append(
                    Wedge(
                        opposite_L1,
                        opposite_L2,
                        X
                    )
                )

    # Finding the Hull Information
    hull_lines = []

    poly = Polygon(X)
    hull = poly.convex_hull
    centroid = np.array((hull.centroid.x, hull.centroid.y))
    hull_coords = np.array(hull.exterior.coords)
    
    for i in range(len(hull_coords) - 1):
        j = i + 1
        A, B, C = line_coeffs(hull_coords[i], hull_coords[j])
        L = Line(A, B, C)
        if centroid[0] * A + centroid[1] * B + C > 0:
            L.sign = -1
        else:
            L.sign = 1
        hull_lines.append(L)

    return boundary_wedges, hull_lines

class AdvBehavior(Agent):
    def __init__(self, node_name):
        self.extra_param_update_map = {
            "Adv.BIG": "big_box",
            "Adv.BisectorDistance": "bisector_distance"
        }
        self.extra_log_field_map = {
            'target': 'target',
            'score': 'score',
            'last_target': 'last_target_replay',
            'last_score': 'last_score',
            'num_compromised': 'num_compromised',
            'targetNeighborhoods': 'targetNeighborhoods',
            'targetNeighborhoodStates': 'targetNeighborhoodStates',
            'wedge_set_lines': 'wedge_set_lines',
            'wedge_set_apex': 'wedge_set_apex',
            'hull_poly': 'hull_poly',
            'jump_blocked': 'jump_blocked',
            'big_box': 'big_box'
        }
   
        super().__init__(node_name)

        self.target = None
        self.score = None
        self.last_target = None
        self.last_target_replay = None
        self.last_score = -np.inf
        self.num_compromised = None

        self.targetNeighborhoods = []
        self.targetNeighborhoodStates = []

        self.wedge_set_lines = None
        # '''
        # [ # For each neighbor hood
        #     [
        #         [A, B, C],  # Line Ax + By + C = 0
        #         [A, B, C]
        #     ]
        # ]
        # '''
        self.wedge_set_apex = None
        # '''
        # [ # For Each Neighbor Hood
        #     [x, y], Apex of 1st
        # ]
        # '''

        self.hull_poly = None
        self.jump_blocked = False

        self.declare_parameter("Adv.BIG", [-10.0, 10.0, -10.0, 10.0])    
        self.big_box = self.get_parameter("Adv.BIG").value if self.get_parameter_or("Adv.BIG", None).type_ != Parameter.Type.NOT_SET else []
        self.big_box = self.big_box

        self.declare_parameter("Adv.BisectorDistance", -1.0)
        self.bisector_distance = self.get_parameter("Adv.BisectorDistance").value
        if self.bisector_distance < 0:
            self.bisector_distance = None
  
    def get_indices(self, robot_thresh=10, neighbor_thresh=2):
        # self._my_neighbors is an array of all the neighbors I know about (Doesn't matter If I consider them in my neighborhood or not) [1,3,4,5,6,7,11,12,16]
        self._my_neighbors = np.array(self._my_neighbors)
        normal_indices = np.where(self._my_neighbors < robot_thresh)[0]
        adversary_indecies = np.where(self._my_neighbors >= robot_thresh)[0]

        # We are assuming that the neighborhood mode is global and that we can see the entire adjacency matrix
        # self._neighborhood has the flatten version of the matrix and self._neighborhood_size has the size
        full_matrix = np.array(self._neighborhood).reshape(self._neighborhood_size, self._neighborhood_size)
        # Need to make sure each row is counting for its own index
        np.fill_diagonal(full_matrix, 1)

        # Lets find how many bad guys are in each row
        bad_count= np.sum(full_matrix[:, adversary_indecies] == 1, axis = 1)
        bad_row_index = np.where(bad_count >= neighbor_thresh)[0]

        # Make sure these are not the rows of the bad agents
        bad_row_index = bad_row_index[~np.isin(bad_row_index, adversary_indecies)]
        bad_rows = full_matrix[bad_row_index]

        return self._my_neighbors[normal_indices], self._my_neighbors[adversary_indecies], bad_rows[:, normal_indices]

    def save_replay_info(self, wedge_sets, hull):
        self.wedge_set_lines = []
        self.wedge_set_apex = []
        for set_idx, wedges in enumerate(wedge_sets):
            self.wedge_set_lines.append([])
            self.wedge_set_apex.append([])
            for wedge_idx, w in enumerate(wedges):
                # Plot the two wedge boundary lines
                self.wedge_set_lines[set_idx].append([])
                self.wedge_set_apex[set_idx].append([float(w.apex[0]), float(w.apex[1])])
                for line_idx, L in enumerate((w.L1, w.L2)):
                    self.wedge_set_lines[set_idx][wedge_idx].append([float(L.A), float(L.B), float(L.C)]) 

        self.hull_poly = np.array(hull.exterior.xy).tolist()
        self.wedge_set_apex = self.wedge_set_apex
        self.wedge_set_lines = self.wedge_set_lines
        self.targetNeighborhoods = np.array(self.targetNeighborhoods).tolist()
        self.targetNeighborhoodStates = np.array(self.targetNeighborhoodStates).tolist()
        
    def controller(self):

       
        # # We do not want his hard coded
        # AdversaryIndices = np.array([11,12,16])
        # NormalIndices = np.array([1,3,4,5,6,7,8,9])
        NormalIndices, AdversaryIndices, self.targetNeighborhoods = self.get_indices()
        self.targetNeighborhoods = self.targetNeighborhoods.tolist()

        X = []
        Y = []
        self.targetNeighborhoodStates = []
        #self.targetNeighborhoods should contain a list of the neighborhoods of normal agents.  It could be just the adjacency matrix. 
        # What would then need to happen is that the adversaries identify vulnerable neighborhoods by comparing the ratio of normal to adversarial neighbors present
        #in the neighborhood.  If there is only one, they just attack the one, if there are multiple, then they look for the best overlap. 
        # self.targetNeighborhoodStates should then be populated with a list of lists of the states of every neighborhood 
        # TargetNeighborhood --- list of lists of indices.    
        # self.targetNeighborhoodStates --- list of lists of np.arrays (states) 
    
        
        for name in NormalIndices:
            if str(name) in self.neighbor_poses:
                X.append(np.array([
                    self.neighbor_poses[str(name)]['pose']['position']['x'],
                    self.neighbor_poses[str(name)]['pose']['position']['y']]))
            else:
                self.get_logger().warning(f"{self.my_name}: Cannot find position for neighbor {name}")
        
        for name in AdversaryIndices:
            if str(name) in self.neighbor_poses:
                Y.append(np.array([
                    self.neighbor_poses[str(name)]['pose']['position']['x'],
                    self.neighbor_poses[str(name)]['pose']['position']['y']])) 
            elif name == self.my_number:
                Y.append(np.array(self.position))
            else:
                self.get_logger().warning(f"{self.my_name}: Cannot find position for neighbor {name}")
        
        for i in range(len(self.targetNeighborhoods)):
            self.targetNeighborhoodStates.append([])
            for index, value in enumerate(self.targetNeighborhoods[i]):
                if value:
                    self.targetNeighborhoodStates[i].append(X[index].tolist())

        # If there are one or  multiple target neighborhoods (multiple neighborhoods with too many adversaries) do this       
        if len(self.targetNeighborhoods)>0:
            wedge_sets = []
            for neighborhood in self.targetNeighborhoodStates:
                boundary_wedges, hull_lines = get_boundary_lines(neighborhood)
                wedge_sets.append(boundary_wedges)

            best = max_color_selection(wedge_sets, X, Y, previous_target=self.last_target, BIG=self.big_box, bisector_distance=self.bisector_distance)
            
            if best is None:
                self.get_logger().warning(f"{self.my_name}: No valid bisector target found.")
                self.move_to_position(self.position)
                
                self.target = None
                self.score = None
                self.last_target = None
                self.last_target_replay = None
                self.last_score = -np.inf
                self.num_compromised = None
                return

            self.num_compromised = int(best[0])
            target = np.array(best[1])
            good_wedges = best[2]
            self.score = float(best[3])
            hull = best[4]

            self.save_replay_info(wedge_sets, hull)
            self.target = target.copy().tolist()
            
            if len(self.target)>0:

                # adding a "sticky" factor to not allow switching of targets easily
                SWITCH_MARGIN = 15.0      
                if self.last_target is not None:
                    jump = np.linalg.norm(target - self.last_target)
                    if jump > 2.0:
                        # Is it really that much better?
                        if self.score < self.last_score + SWITCH_MARGIN:
                            # Keep chasing the old target
                            target = self.last_target.copy()
                            self.score = self.last_score
                            self.get_logger().info(f"Jump Margin was met and we decided NOT to Jump. Old score: {self.last_score} New Score: {self.score} ({self.score - self.last_score})")
                            self.jump_blocked = True
                        else:
                            self.get_logger().info(f"Jump Margin was met and we decided to Jump. Old score: {self.last_score} New Score: {self.score} ({self.score - self.last_score})")
                            self.jump_blocked = False
                    else:
                        self.jump_blocked = False

                self.last_target = target.copy()
                self.last_target_replay = self.last_target.tolist()
                self.last_score = self.score
                self.move_to_position(target)
            else: 
                self.get_logger().warning(f"{self.my_name}: ERROR - Target had length of 0")
                self.move_to_position(self.position)
        # If there is no target neighborhood
        else:
            self.move_to_position(self.position)
        
        

def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=2
    ## ros2 run agent_control doDriveAway.py --ros-args -p robot.id:=2 -p robot.neighbors:='[1,2]' -p mode.sim:=true 

    my_robot = None 

    try:
        rclpy.init(args=args)
        my_robot = AdvBehavior("AdvBehavior")
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        if my_robot:
            my_robot.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()