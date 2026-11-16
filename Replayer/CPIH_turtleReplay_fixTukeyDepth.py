import json
import zipfile
import os
from copy import deepcopy
from os import listdir, remove, rmdir, path, getcwd
import tkinter as tk
from tkinter import filedialog
import numpy as np
from itertools import combinations
from pathlib import Path
import shutil
import pdb

uncompress_file = None

# =========================
# Load Replay from Zip
# =========================
def load_replay_zip(zip_path, extract_dir="usable_replay"):
    """
    Extracts a replay zip and loads JSON lines into a list.
    """
    global uncompress_file

    data = []
    with zipfile.ZipFile(zip_path, "r") as zip_ref:
        uncompress_file = zip_ref.namelist()[0]

        with zip_ref.open(uncompress_file) as curFile:
            for line in curFile:
                line = line.decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                data.extend(json.loads(line))

    return data


# =========================
# Fix Entire Replay Dataset
# =========================
def recompute_tukey_depths(data, verbose=False, file_name=None):
    """
    This function will look loop through the data if if current tukey depth is not Null, we will recompute it.

    My position is in my_pose.pose.position and my neighbors can be found in neighbor_position.
    self_trust:
        0: Distrust (not used)
        1: Normal
        2: Self Trust
    """
    updated_count = 0
    for entry in data:
        # If not tukey depth was calculated for this time step, it will show 0
        if 'tukey_depth' in entry and entry['tukey_depth']:
            old_depth = entry['tukey_depth']
            
            # Build X
            x, y, _ = entry['my_pose']['pose']['position'].values()
            X = np.zeros((len(entry['neighbor_position'])+1,2))
            i = 1
            X[0] = [x, y]
            for name, neighbor in entry['neighbor_position'].items():
                X[i] = np.array(neighbor) 
                # print("X[",i,"]: ",X[i])
                i = i+1

            # Check what trust mode we are doing

            if entry['self_trust'] == 0:
                X = np.delete(X,0,axis = 0)
                _, center_depth, tukey_depth, _ = get_tukeyDepth(X)
            if entry['self_trust'] == 1:
                _, center_depth, tukey_depth, _ = get_tukeyDepth(X)
            if entry['self_trust'] == 2:
                _, center_depth, tukey_depth, _ = get_selfTrustTukeyDepth(X,0)

            new_depth = float(tukey_depth)
            if new_depth != old_depth:
                entry['tukey_depth'] = new_depth
                updated_count = updated_count + 1

    if verbose:
        if updated_count:
            print(f"*****************\n{file_name}\nUpdated {updated_count} Tukey Depths\n*****************")
        else:
            print("*****************\nNo Values Updated\n*****************")

    return data, updated_count > 0

# ========================================================================
# Function pulled from TukeyMedian.py to calculate tukey depth. (TukeyMed)
# ========================================================================
def get_tukeyDepth(X, specK = -1):
    """
    X: nx2 matrix
    """
    n = len(X)
    # Store tuples of (k, a, b, c) representing constraint ax + by + c <= 0
    candidate_constraints = []
    
    indices = range(n)
    epsilon = 1e-9

    center_depth = np.ceil(len(X)*1/3)
    tukey_depth_found = False
    tukey_depth = 0
    
    # 1. Loop through all pairs
    for idx1, idx2 in combinations(indices, 2):
        p1 = X[idx1]
        p2 = X[idx2]
        
        # Line formulation: A*x + B*y + C = 0
        dy = p2[1] - p1[1]
        dx = p2[0] - p1[0]
        a = -dy
        b = dx
        c = - (a * p1[0] + b * p1[1])
        
        # Evaluate points
        vals = a * X[:, 0] + b * X[:, 1] + c
        
        n_above = np.sum(vals > epsilon)
        n_below = np.sum(vals < -epsilon)
        

        k = min(n_above, n_below)

        if n_above < n_below:
            # Constraint: val <= 0 -> ax + by + c <= 0
            candidate_constraints.append((k, a, b, c))
            
        # Case B: Below is smaller -> Keep ABOVE
        elif n_below < n_above:
            # Constraint: val >= 0 -> -val <= 0 -> -ax -by -c <= 0
            candidate_constraints.append((k, -a, -b, -c))
            
        # Case C: Equal parts
        else:

            candidate_constraints.append((k, a, b, c))   # Keep Below
            candidate_constraints.append((k, -a, -b, -c)) # Keep Above

    if not candidate_constraints:
        print("No valid constraints found.")
        return 0

    # 4. Find the highest k
    max_k = max(item[0] for item in candidate_constraints)
    if specK > 0:
        set_k = specK-1
    else:
        set_k = max_k

    median_found = False
    final_poly = np.array([]) 
    while (not median_found) and (max_k >= 0):
    # Filter constraints for this max k
        active_constraints = [item for item in candidate_constraints if item[0] == max_k]
        
        # 5. Compute Intersection
        inf = 1e9
        poly = np.array([[-inf, -inf], [inf, -inf], [inf, inf], [-inf, inf]])
        
    
        for _, a, b, c in active_constraints:
            poly = clip_polygon(poly, a, b, c)
        if len(poly) > 0:
            if not tukey_depth_found:
                tukey_depth_found = True
                tukey_depth = max_k + 1

            if max_k <= set_k:
                median_found = True
                final_poly = poly
            else:
                max_k = set_k
        else:
            max_k = max_k-1
       
    return max_k+1, center_depth, tukey_depth, final_poly

# ============================================================================================
# Function pulled from TukeyMedian.py to calculate tukey depth in self trust mode. (ZTukeyMed)
# ============================================================================================
def get_selfTrustTukeyDepth(X, i, specK = -1):
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

    center_depth = np.ceil(len(X)*1/3)
    tukey_depth_found = False
    tukey_depth = 0
    
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

    if specK > 0:
        set_k = specK-1
    else:
        set_k = max_k

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
                if not tukey_depth_found:
                    tukey_depth_found = True
                    tukey_depth = max_k + 1

                if max_k <= set_k:
                    median_found = True
                    final_poly = poly
                else:
                    max_k = set_k
            else:
                max_k -= 1
        else:
            max_k -= 1
    
    return max_k+1, center_depth, tukey_depth, final_poly

# ===============================================================================
# Function pulled from TukeyMedian.py. Support function to calculate Tukey Median
# ===============================================================================
def clip_polygon(poly, a, b, c, d=0):
    new_poly = []
    if len(poly) == 0: return new_poly
    
    # d=1: Intersect with line ax+by+c=0
    if d == 1:
        for j in range(len(poly)):
            p_curr = poly[j]
            p_next = poly[(j + 1) % len(poly)]
            
            val_curr = a * p_curr[0] + b * p_curr[1] + c
            val_next = a * p_next[0] + b * p_next[1] + c
            
            # If point is on the line, keep it
            if abs(val_curr) <= 1e-9:
                new_poly.append(p_curr)
                
            # If edge crosses the line (strict crossing), add intersection
            if (val_curr < -1e-9 and val_next > 1e-9) or \
                (val_curr > 1e-9 and val_next < -1e-9):
                denom = val_curr - val_next
                if abs(denom) > 1e-12:
                    t = val_curr / denom
                    inter_p = p_curr + t * (p_next - p_curr)
                    new_poly.append(inter_p)
        return np.array(new_poly)

    # d=0: Clip against halfspace ax+by+c <= 0
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

# =========================
# Save Replay (JSON lines)
# =========================
def save_replay(data, output_file=None, output_dir=None):
    """
    Saves replay data as JSON lines (same format as original).
    """
    global uncompress_file

    if not output_file:
        output_file = uncompress_file

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, output_file)

    with open(output_file, 'w') as f:
        for entry in data:
            f.write(json.dumps(entry) + "\n")

    return output_file


# =========================
# Save Replay as Zip
# =========================
def save_replay_zip(data, output_dir=None, zip_name=None, temp_file=None):
    """
    Saves replay data and compresses into a zip file.
    """
    global uncompress_file

    if not zip_name:
        zip_name = uncompress_file.split('.')[0] + ".turtleReplay"
    if not temp_file:
        temp_file = uncompress_file

    # Ensure directory exists
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        zip_path = os.path.join(output_dir, zip_name)
    else:
        zip_path = zip_name

    # Build JSON
    replay_contents = "\n".join(
        "[" + json.dumps(entry) + "]"
        for entry in data
    )

    # Zip it
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        zipf.writestr(
            temp_file,
            replay_contents
            )
    return zip_path


# =========================
# One-shot Utility Function
# =========================
def load_fix_and_save(zip_input, zip_output=None, output_dir=None, verbose=False, save=True):
    """
    Full pipeline:
    Load → Fix → Save (optionally zipped)
    """
    data = load_replay_zip(zip_input)
    fixed_data, edited = recompute_tukey_depths(data, verbose=verbose, file_name=zip_input)

    if save and edited:
        if zip_output:
            save_replay_zip(fixed_data, zip_name=zip_output, output_dir=output_dir)
        else:
            save_replay(fixed_data, output_dir=output_dir)

    return fixed_data

def delete_fixed_directories(parent_dir):
    """
    Recursively delete all directories named 'fixed'
    starting from parent_dir.
    """
    parent_dir = Path(parent_dir)

    deleted_count = 0

    for directory in parent_dir.rglob("fixed"):
        if directory.is_dir():
            print(f"Deleting: {directory}")
            shutil.rmtree(directory)
            deleted_count += 1

    print(f"\nDeleted {deleted_count} fixed directories.")

delete_fixed = False
if __name__ == "__main__":

    if not delete_fixed:
        # # Used to manually pick a file
        # root = tk.Tk()
        # root.withdraw()

        # # Default directory logic
        # start_path = path.abspath(path.join(getcwd(), "..", "Replays"))
        # if not path.exists(start_path):
        #     start_path = path.abspath(path.join(getcwd(), "Replays"))

        # replay_files = filedialog.askopenfilenames(
        #     title="Select Replay file(s)",
        #     initialdir=start_path,
        #     filetypes=[("Replay Files", "*.turtleReplay")]
        # )

        # root.destroy()

        # if not replay_files:
        #     print("No files selected.")
        #     exit()

        # Looping through all the experiments
        root = Path("/Users/jss150630/Programs/VM/UbuntuJellyfish/SharedDrive/turtlebot_simulator/Replays/CPIH_Paper/Experiments")
        for file_path in root.rglob("*.turtleReplay"):
        # for file_path in replay_files:
            print(f"Processing: {file_path}")

            # Example: save into "fixed" subfolder next to original file
            # output_dir = path.join(path.dirname(file_path), "fixed")
            output_dir = path.dirname(file_path)

            load_fix_and_save(
                file_path,
                zip_output=path.basename(file_path),
                output_dir=output_dir,
                verbose=True
            )

        print("Done processing all files.")

    else:
        # Deleting Fixed Directories

        root = Path("/Users/jss150630/Programs/VM/UbuntuJellyfish/SharedDrive/turtlebot_simulator/Replays/CPIH_Paper/Experiments")
        delete_fixed_directories(root)