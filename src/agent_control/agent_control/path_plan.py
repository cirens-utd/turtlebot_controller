import numpy as np
import heapq
# Parameters

class Path():
    def __init__(self, my_position= (0,0), robot_positions= [(0,0)],goal_position = (0,0)):
        self.map_size = (9, 9)  # Grid dimensions (e.g., 100x100 cells)
        self.resolution = 0.5  # Size of each grid cell (meters)
        self.costmap = np.zeros(map_size)
        self.robot_positions = robot_positions
        self.goal_position = goal_position
        # Constants
        self.SAFE_RADIUS = 0.5  # Safe distance (meters) around each robot
        self.MAX_COST = 255  # Maximum cos
        self.start = my_position
    def world_to_grid(self,x, y, resolution):
        return int(x / resolution), int(y / resolution)
    def grid_to_world(self,x,y):
        return x * self.resolution, y * self.resolution
    def update_costmap(self, robot_positions):
        costmap = self.costmap.copy()
        for x, y in robot_positions:
            cx, cy = self.world_to_grid(x, y, self.resolution)
            radius_cells = int(self.SAFE_RADIUS / self.resolution)
            
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    gx, gy = cx + dx, cy + dy
                    if 0 <= gx < costmap.shape[0] and 0 <= gy < costmap.shape[1]:
                        dist = np.hypot(dx, dy) * resolution
                        if dist <= safe_radius:
                            costmap[gx, gy] = self.MAX_COST
        self.costmap = costmap
       
    def astar(self, start, goal):
        start = self.world_to_grid(start,self.resolution)
        goal = self.world_to_grid(goal,self.resolution)
        open_set = []
        heapq.heappush(open_set, (0 + np.linalg.norm(np.array(start) - np.array(goal)), 0, start, [start]))
        visited = set()
        
        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            if current == goal:
                for grid_point in path:
                    grid_point = grid_point*self.resolution
                return path
            if current in visited:
                continue
            visited.add(current)

            x, y = current
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.costmap.shape[0] and 0 <= ny < self.costmap.shape[1] and self.costmap[nx, ny] < self.MAX_COST:
                    heapq.heappush(open_set, (
                        cost + 1 + np.linalg.norm(np.array([nx, ny]) - np.array(goal)),
                        cost + 1,
                        (nx, ny),
                        path + [(nx, ny)]
                    ))
        return None  # No path found


# robot_positions = [(1.0, 1.0), (1.5, 1.5), ..., (3.0, 3.5)]  # Example coordinates
# goal_position = (8.0, 8.0)

# costmap = np.zeros(map_size)
# costmap = update_costmap(costmap, robot_positions, SAFE_RADIUS, resolution)

# for i, robot_pos in enumerate(robot_positions):
#     start_grid = world_to_grid(*robot_pos, resolution)
#     goal_grid = world_to_grid(*goal_position, resolution)
    
#     path = astar(costmap.copy(), start_grid, goal_grid)
#     print(f"Robot {i} path: {path}")