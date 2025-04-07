### Calculate Intersections

## Generate Slicing Planes

# Import necessary packages
import time
import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
import pprint

from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d .art3d import Poly3DCollection

start = time.time()

### ------- USER INPUTS -------- ###
layer_height = 1 # Predetermined by extruder/nozzle settings
res = 25 # Resolution is for how many points to use along x-y plane when making a surface

cube = mesh.Mesh.from_file('/Users/adityarao/Desktop/Slicing Algorithm/STL Files/Pyramid_Complex.STL') # Opening STL and reading as mesh
### ------- USER INPUTS -------- ###

# Pulling out the data from the STL
facet_data = []
for i in range(len(cube.vectors)):
    facet = [i, 
    cube.v0[i].tolist(),
    cube.v1[i].tolist(), 
    cube.v2[i].tolist(),
    cube.normals[i].tolist()]
    
    facet_data.append(facet)
    ## FACET_DATA = # ; v0 ; v1 ; v2 ; norm

# Pulling out the max and min values for (x,y,z)
# helps to bound the plane we need to make in the x-y directions 
# and define the upper and lower bounds for the z plane heights
x_max, y_max, z_max = cube.max_
x_min, y_min, z_min = cube.min_

# Extracting the vertices and normals data
verts = cube.vectors
normals = cube.normals

# Generate arrays for x and y that go from min to max with res # of points, make into grid
x = np.linspace(x_min,x_max,res)
y = np.linspace(y_min,y_max,res)
X, Y = np.meshgrid(x,y)

# Graphing Code
fig = plt.figure()
axes = fig.add_subplot(111, projection='3d')

# Make a list z_vals that goes from z min to max and goes up by layer height intervals
# might create an extra plane with nothing
z_planes = np.arange(z_min,z_max+layer_height,layer_height)

# Loop through z_vals to generate and plot each z-plane
for i in z_planes:
    Z = np.full_like(X,i)
    axes.plot_surface(X, Y, Z, color='red', alpha=0.7, edgecolor='k')

# Graph STL and scale
axes.add_collection3d(Poly3DCollection(verts, facecolors='blue', linewidths=1.5, edgecolor='k', alpha=0.2))
#scale = cube.points.flatten()
#axes.auto_scale_xyz(scale, scale, scale)
axes.set_xlabel("X-axis")
axes.set_ylabel("Y-axis")
axes.set_zlabel("Z-axis")
axes.set_title("3D Visualization of Slicing Planes on STL")

## ------ STL Extraction/Slicing Planes Return Values ------- ##
print("======== Slicing Algorithm Outputs ========")
#print(z_planes) # list of the z-values for the planes generated
print("Number of Slicing Planes:", len(z_planes),"\n")
#pprint.pprint(facet_data)
print("Number of Facets:", len(facet_data),"\n")

#plt.show() ### PLOT STL + PLANES

##### ------ Calculation of Intersections ------- #####

# Defining variables
z_planes = z_planes
facets = facet_data ## FACET_DATA = # ; v0 ; v1 ; v2 ; norm

intersections = [] # List of ALL intersections
# Loop through z_planes
for z_plane in z_planes:
    slice_plane = [] # List of intersections per plane
    # Loop through all facets
    for facet in facets:
        # Make a list with the vertices
        v_list = [facet[1], facet[2], facet[3]]

        # Categorize the points based on z height relative to current z_plane and place it into appropriate list
        v_above, v_below, v_on = [], [], [] # 3 empty lists

        for v in v_list:
            if v[2] > z_plane:
                v_above.append(v)
            elif v[2] < z_plane:
                v_below.append(v)
            #elif v[2] == z_plane:
            elif abs(v[2]-z_plane) < 1e-3: # Tolerance
                v_on.append(v)

        ## Use the categorized vertices to loop through some Cases
        # Need 2 Intersection Points for each case

        # Case 1: 2 points above and 1 below
        if len(v_above) == 2 and len(v_below) == 1:
            # Assign points 1-3 for calculations
            p1 = v_above[0]
            p2 = v_above[1]
            p3 = v_below[0]

        # Case 2: 2 points below and 1 above
        elif len(v_above) == 1 and len(v_below) == 2:
            # Assign points 1-3 for calculations
            p1 = v_below[0]
            p2 = v_below[1]
            p3 = v_above[0]
        
        # Calculations for Case 1 and 2 done separately

        # Case 3: 1 above, 1 below, 1 on plane
        elif len(v_on) == 1 and len(v_above) == 1 and len(v_below) == 1:
            int1 = v_on[0] # INTERSECTION 1
            # Assign points for calculations
            p1 = v_above[0]
            p2 = v_below[0]

            # Calculations
            # Interpolation
            z_term = (z_plane - p2[2])/(p1[2] - p2[2])
            # Adding to x/y
            x2 = p2[0] + z_term*(p1[0] - p2[0])
            y2 = p2[1] + z_term*(p1[1] - p2[1])
            int2 = [x2, y2, z_plane] # INTERSECTION 2
            # Add intersections to slice plane intersection
            slice_plane.append((tuple(int1), tuple(int2)))
            continue

        # Case 4: 2 points on plane
        elif len(v_on) == 2:
            int1 = v_on[0]
            int2 = v_on[1]
            # Add intersections to slice plane intersection
            slice_plane.append((tuple(int1), tuple(int2)))
            continue

        # Case 5: All 3 points on plane
        elif len(v_on) == 3:
            #int1 = v_on[0]
            #int2 = v_on[1]
            #int3 = v_on[2]

            #slice_plane.append((tuple(int1), tuple(int2)))
            #slice_plane.append((tuple(int1), tuple(int3)))
            #slice_plane.append((tuple(int3), tuple(int2)))
            
            for i in range(3):
                slice_plane.append((tuple(v_on[i]), tuple(v_on[(i + 1) % 3]))) 
            continue
        
        # Case 6: Facet does not intersect current z_plane
        else:
            continue

        # Calculations for Case 1 and 2
        # Convert points to arrays
        #p1 = np.array(p1)
        #p2 = np.array(p2)
        #p3 = np.array(p3)

        # Interpolate + Add to x/y values
        z_term1 = (z_plane - p1[2])/(p3[2] - p1[2])
        x1 = p1[0] + z_term1*(p3[0] - p1[0])
        y1 = p1[1] + z_term1*(p3[1] - p1[1])
        int1 = [x1, y1, z_plane] # INTERSECTION 1

        z_term2 = (z_plane - p2[2])/(p3[2] - p2[2])
        x2 = p2[0] + z_term2*(p3[0] - p2[0])
        y2 = p2[1] + z_term2*(p3[1] - p2[1])
        int2 = [x2, y2, z_plane] # INTERSECTION 2
        # Add intersections to slice plane intersection
        slice_plane.append((tuple(int1), tuple(int2)))

    # Add the current plane and the intersections for plane in the big list
    intersections.append([z_plane, slice_plane])


## ------ Data Return Values ------- ##

# Intersections is a list of lists with the z_plane and intersections
for i in intersections: # Loop through each slice in intersections
    z_plane = i[0] # extract z_plane
    ints= i[1] # exttact ints
    print("Z =", z_plane, "has ", len(ints), "intersections") # Print statement
    #for j in ints: # print the intersections
        #pprint.pprint(j)

# List of Intersections
# Essentially a list for each plane has the z_plane and tuples that form an intersection line, one list per plane
pprint.pprint(intersections)

## ------ Plotting ------- ##
# Create a new figure
fig2 = plt.figure()
axes2 = fig2.add_subplot(111, projection='3d')

# Plot the intersection lines in black
for plane in intersections:
    z_val = plane[0]  # Z height of the slicing plane
    segments = plane[1]  # List of intersection segments
    
    for seg in segments:
        p1, p2 = seg  # Each segment contains two points
        x_vals = [p1[0], p2[0]]
        y_vals = [p1[1], p2[1]]
        z_vals = [p1[2], p2[2]]
        axes2.plot(x_vals, y_vals, z_vals, 'b-', linewidth=2)  # Black lines

# Set labels and title
axes2.set_xlabel("X-axis")
axes2.set_ylabel("Y-axis")
axes2.set_zlabel("Z-axis")
axes2.set_title("3D Visualization of Intersections")

# Adjust the viewing angle for better perspective
axes2.view_init(elev=30, azim=45)  # Adjust elevation and azimuth for a better view

# Creating a new plot for STL only
fig3 = plt.figure()
axes3 = fig3.add_subplot(111, projection='3d')
axes3.add_collection3d(Poly3DCollection(verts, facecolors='blue', linewidths=1.5, edgecolor='k', alpha=0.2))
axes3.set_xlabel("X-axis")
axes3.set_ylabel("Y-axis")
axes3.set_zlabel("Z-axis")
axes3.set_title("3D Visualization of STL")
plt.show()

# Time duration of code running calculation
end = time.time()
print("\nTime elasped = ", (end-start), "seconds")