import matplotlib.pyplot as plt
import numpy as np

# List to store the coordinates of the clicks
click_coords = []

def onclick(event):
    # If the Escape key is pressed, stop interaction
    if event.key == 'escape':
        plt.close()
    # Otherwise, store the coordinates of the click
    else:
        click_coords.append((event.xdata, event.ydata))
        print(f"Clicked at: {event.xdata:.2f}, {event.ydata:.2f}")
        
        # Plot the point as it is clicked
        ax.scatter(event.xdata, event.ydata, color='blue')
        ax.set_xlim(0, 6)
        ax.set_ylim(0, 6)
        plt.draw()

# Create a figure and a plot
fig, ax = plt.subplots()
ax.set_title('Click on the plot (Press Escape to finish)')
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)

# Connect the click event to the 'onclick' function
cid = fig.canvas.mpl_connect('button_press_event', onclick)

# Display the plot and wait for user interaction
plt.show()

# Print all the recorded coordinates after the plot is closed
print("\nAll recorded clicks (coordinates):")
for coord in click_coords:
    print(f"({coord[0]:.2f}, {coord[0]:.2f})")

# Calculate the pairwise Euclidean distances and store them in a matrix
if len(click_coords) > 1:
    coords_array = np.array(click_coords)
    num_points = coords_array.shape[0]
    dist_matrix = np.zeros((num_points, num_points))

    for i in range(num_points):
        for j in range(num_points):
            dist_matrix[i, j] = np.linalg.norm(coords_array[i] - coords_array[j])

    # Print the distance matrix
    print("\nPairwise Distance Matrix:")
    print("[", end ='')
    for idx, row in enumerate(dist_matrix):
        print("[" + ", ".join(f"{val:.2f}" for val in row) + "]",end='')
        if idx+1 != len(dist_matrix):
            print(',', end='')

    print(']')
else:
    print("Not enough points to calculate distances.")
