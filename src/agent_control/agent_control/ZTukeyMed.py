import numpy as np
from itertools import combinations

def ZTukeyMed(X, i):
    """
    X: nx2 matrix 
    i: index of z
    plot: boolean, plot == True: generates plot. plot == false: no plot, just returns depth and z-median vertices
 
    """
    z = X[i]
    n = len(X)
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
    max_k = max(item[0] for item in candidate_constraints)
    median_found = False
    final_poly = np.array([]) 

    while (not median_found) and (max_k >= 0):
        active_constraints = [item for item in candidate_constraints if item[0] <= max_k]
        
        inf = 1e9
        poly = np.array([[-inf, -inf], [inf, -inf], [inf, inf], [-inf, inf]])
        
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

  
    
    return max_k+1, final_poly