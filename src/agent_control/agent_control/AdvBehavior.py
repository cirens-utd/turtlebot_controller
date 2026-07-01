
#!/usr/bin/env python3

import rclpy
from agent_control.agent import Agent
import numpy as np
import traceback
import pdb
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from shapely import Polygon,Point,MultiPoint,LineString

from itertools import combinations

class AdvBehavior(Agent):
    def __init__(self, node_name):
   
        super().__init__(node_name)

     
    class Line:
        def __init__(self,a,b,c, sign = 0):
            norm = np.sqrt(a*a+b*b)
            self.A = a/norm
            self.B = b/norm
            self.C = c/norm
            self.sign = sign
    def dist_to_line(self,pt, line):
        return abs(line.A*pt[0] + line.B*pt[1] + line.C)
    def line_intersection(self,L1, L2):
        A = np.array([[L1.A, L1.B],
                    [L2.A, L2.B]])
        b = -np.array([L1.C, L2.C])
        if abs(np.linalg.det(A)) < 1e-10:
            return None

        return np.linalg.solve(A,b)
    def line_coeffs(self,p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        A = y2 - y1
        B = x1 - x2
        C = x2*y1 - x1*y2
        return A, B, C
    def angle_bisectors(self,L1, L2):

        # line directions
        d1 = np.array([-L1.B, L1.A])
        d2 = np.array([-L2.B, L2.A])

        d1 /= np.linalg.norm(d1)
        d2 /= np.linalg.norm(d2)
        bisectors = []
        for b in (d1+d2, d1-d2):

            if np.linalg.norm(b) < 1e-10:
                continue

            b /= np.linalg.norm(b)

            bisectors.append(b)
            bisectors.append(-b)

        return bisectors
    def get_projected_pos(self,Y, target_line, hull_lines =[], buffer = 1.0):
        v = np.array([-target_line.A,target_line.B])
        vdir = v/np.linalg.norm(v)
        p = np.array([0, -target_line.C/target_line.B])
        proj_positions = []
        m = len(Y)
        outside_hull = np.zeros(m)
        intersections = []
        first_val = 10000
        first_point = []
        for y in Y:
            u = y-p
            t = np.dot(u,v)/np.dot(v,v)
            if t<first_val:
                first_val = t
                first_point = y
            proj_positions.append(p+t*v)
        for line in hull_lines:
            intersect = self.line_intersection(target_line, line)
            dist = np.linalg.norm(intersect-pos)
            intersections.append([intersect, dist])
            i = 0
            for pos in proj_positions:
                if  line.sign*(line.A*pos[0]+line.B*pos[1]+line.C)>0:
                    outside_hull[i] = 1
                i+=1
        if  sum(outside_hull) == 0:
            intersections = sorted(intersections, key=lambda x: x[1])
            for point, dist in intersections:
                t_v = (point[0]-first_point[0])/vdir[0]
                if t_v>0:
                    for pos in proj_positions:
                        pos +=(dist+buffer)*vdir
        return proj_positions
    def get_boundary_lines(self,X):
        n = len(X)
        N = np.arange(n);
        pairs = list(combinations(N,2));
        boundary_lines = []
        hull_lines = []
        for pair in pairs:
            A, B, C = self.line_coeffs(X[pair[0]], X[pair[1]])
            total = 0
            for point in X:
                total= total+self.side(A, B, C, point)
            if total != 0:
                continue
            else:
                L = self.Line(A,B,C)
                boundary_lines.append(L)
        poly = Polygon(X)
        hull = poly.convex_hull
        centroid = np.array((hull.centroid.x,hull.centroid.y))
        hull = np.array(hull.exterior.coords)
        
        for i in range(len(hull)-1):
            j = i+1
            A,B,C = self.line_coeffs(hull[i],hull[j])
            L = self.Line(A,B,C)
            if centroid[0]*A+centroid[1]*B+C>0:
                L.sign = -1
            else:
                L.sign = 1
            hull_lines.append(L)
            
        return boundary_lines,hull_lines
    def controller(self):
        '''
        This function is called every time the robot position is updated. We will put our concensus logic here.

        Equation:
        drive away from concensus

        Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position
        '''
        #Need to define these parameters
        X = []
        Y=[] 
        my_idx = 0
        boundary_lines, hull_lines = self.get_boundary_lines(X)
        line_pairs = list(combinations(np.arange(len(boundary_lines)),2))
        bisectors = []
        for pair in line_pairs:
            bisectors.append(self.angle_bisectors(boundary_lines[pair[0]],boundary_lines[pair[1]]))

        best_targets = []
       
        best_dist = 1000000
        for line in bisectors:
            projected_targets = self.get_projected_pos(Y,line,hull_lines)
            dist = 0
            for i in range(len(Y)):
                dist += np.linalg.norm(Y[i]-projected_targets[i])
            if dist< best_dist:
                best_targets = projected_targets
                best_dist = dist
       # for name, neighbor in self.neighbor_position.items():
        #    difference = (np.array(neighbor) - np.array(self.position))/2
         #   total +=  difference
        my_target = best_targets[my_idx]-self.position
        #REPLACE WITH CORRECT FUNCTION
        self.MOVETOPOINT(my_target)


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
