import heapq
import numpy as np # Import numpy

class Path():
    def __init__(self, my_position= (0,0), robot_positions= [(0,0)],goal_position = (0,0)):
        self.map_size = (9, 9)  # Grid dimensions (e.g., 100x100 cells)
        self.resolution = 0.5  # Size of each grid cell (meters).  So the map is covers a 4.5x4.5 area in worldspace
        self.costmap = np.zeros(self.map_size)
        self.robot_positions = robot_positions
        self.goal_position = goal_position
        # Constants
        self.SAFE_RADIUS = 0.5  # Safe distance (meters) around each robot
        self.radius_cells = int(self.SAFE_RADIUS / self.resolution) 
        self.MAX_COST = 255  # Maximum cost
        self.start = my_position

    def world_to_grid(self,x, y, resolution):
        return int(x / resolution), int(y / resolution)

    def grid_to_world(self,x,y):
        return x * self.resolution, y * self.resolution

    def update_costmap(self, robot_positions):
        costmap = self.costmap.copy()
        for x, y in robot_positions:
            cx, cy = self.world_to_grid(x, y, self.resolution)
            
            for dx in range(-self.radius_cells, self.radius_cells + 1):
                for dy in range(-self.radius_cells, self.radius_cells + 1):
                    gx, gy = cx + dx, cy + dy
                    if 0 <= gx < costmap.shape[0] and 0 <= gy < costmap.shape[1]:
                        dist = np.hypot(dx * self.resolution, dy * self.resolution) # Multiply by resolution for true distance
                        if dist <= self.SAFE_RADIUS:
                            costmap[gx, gy] = self.MAX_COST
        self.costmap = costmap
        
    def astar(self, start, goal):
        start_grid = self.world_to_grid(start[0],start[1],self.resolution)
        goal_grid = self.world_to_grid(goal[0],goal[1],self.resolution)

        # Check if start or goal are within map bounds or are obstacles
        if not (0 <= start_grid[0] < self.costmap.shape[0] and 0 <= start_grid[1] < self.costmap.shape[1]):
            err = f"Start position {start} is out of map bounds {self.map_size*self.resolution}."
            return err
        if not (0 <= goal_grid[0] < self.costmap.shape[0] and 0 <= goal_grid[1] < self.costmap.shape[1]):
            err = f"Goal position {goal} is out of map bounds {self.map_size*self.resolution}."
            return err
        if self.costmap[start_grid[0], start_grid[1]] >= self.MAX_COST:
            err = f"Start position {start} is an obstacle."
            return err
        if self.costmap[goal_grid[0], goal_grid[1]] >= self.MAX_COST:
            err = f"Goal position {start} is an obstacle."
            return err


        open_set = []
        heapq.heappush(open_set, (0 + np.linalg.norm(np.array(start_grid) - np.array(goal_grid)), 0, start_grid, [start_grid]))
        visited = set()
        
        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)

            if current == goal_grid:
                world_path = [(grid_point[0] * self.resolution, grid_point[1] * self.resolution) for grid_point in path]
                return world_path
            
            if current in visited:
                continue
            visited.add(current)

            x, y = current
            #for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]: # 4-directional movement
            # For 8-directional movement, uncomment the line below and comment the line above:
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.costmap.shape[0] and 0 <= ny < self.costmap.shape[1] and self.costmap[nx, ny] < self.MAX_COST:
                    new_cost = cost + 1 # Uniform cost for cardinal movements
                    # If 8-directional, diagonal moves might have cost sqrt(2)
                    if abs(dx) + abs(dy) == 2: # Diagonal move
                         new_cost = cost + np.sqrt(2)

                    heuristic = np.linalg.norm(np.array([nx, ny]) - np.array(goal_grid))
                    heapq.heappush(open_set, (
                        new_cost + heuristic,
                        new_cost,
                        (nx, ny),
                        path + [(nx, ny)]
                    ))
        return "No path found"

