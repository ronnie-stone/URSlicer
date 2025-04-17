# from slicing_configs_v4 import config
import time
import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
import pprint

from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from collections import defaultdict

from numpy.linalg import inv
from scipy.spatial import ConvexHull


class STL2Motion:
    """Fill in comment"""

    def __init__(self, filepath_to_stl, quaternion):
        self.filepath = filepath_to_stl
        self.input_quaternion = quaternion

        self.config = {
            "layer_height": 5.0,  # Predetermined by extruder/nozzle settings
            "resolution": 25,  # Resolution is for how many points to use along x-y plane when making a surface
            "infill_angle": 90,  # Rotation from x-axis for infill -- DEGREES
            "infill_density": 0.1,  # Density of infill (0-1) - Spacing of lines with respect to nozzle dia
            "nozzle_dia": 0.5,  # Predetermined by extruder/nozzle settings
        }

    def generate_robot_points(self):
        """Fill in comment"""
        # Example function

        cube = mesh.Mesh.from_file(self.filepath)

        layer_height = self.config["layer_height"]
        res = self.config["resolution"]
        infill_angle = self.config["infill_angle"]
        infill_density = self.config["infill_density"]
        nozzle_dia = self.config["nozzle_dia"]
        tolerance = 1e-6
        nozzle_half = nozzle_dia / 2

        facet_data = []
        for i in range(len(cube.vectors)):
            facet = [
                i,
                cube.v0[i].tolist(),
                cube.v1[i].tolist(),
                cube.v2[i].tolist(),
                cube.normals[i].tolist(),
            ]

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
        x = np.linspace(x_min, x_max, res)
        y = np.linspace(y_min, y_max, res)
        X, Y = np.meshgrid(x, y)

        # Make a list z_vals that goes from z min to max and goes up by layer height intervals
        # might create an extra plane with nothing
        z_planes = np.arange(z_min, z_max + layer_height, layer_height)

        ##### ------ Calculation of Intersections ------- #####

        # Defining variables
        z_planes = z_planes
        facets = facet_data  ## FACET_DATA = # ; v0 ; v1 ; v2 ; norm

        z_model = z_max - z_min

        intersections = []  # List of ALL intersections
        # Loop through z_planes
        for z_plane in z_planes:
            slice_plane = []  # List of intersections per plane
            # Loop through all facets
            for facet in facets:
                # Make a list with the vertices
                v_list = [facet[1], facet[2], facet[3]]
                v_norm = facet[4]

                rel_tol = max(1e-6, 0.001 * z_model)

                # Categorize the points based on z height relative to current z_plane and place it into appropriate list
                v_above, v_below, v_on = [], [], []  # 3 empty lists

                for v in v_list:
                    if v[2] > z_plane + rel_tol:
                        v_above.append(v)
                    elif v[2] < z_plane - rel_tol:
                        v_below.append(v)
                    # elif v[2] == z_plane:
                    elif abs(v[2] - z_plane) < 1e-6:  # Tolerance
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
                    int1 = v_on[0]  # INTERSECTION 1
                    # Assign points for calculations
                    p1 = v_above[0]
                    p2 = v_below[0]

                    # Calculations
                    # Interpolation
                    z_term = (z_plane - p2[2]) / (p1[2] - p2[2])
                    # Adding to x/y
                    x2 = p2[0] + z_term * (p1[0] - p2[0])
                    y2 = p2[1] + z_term * (p1[1] - p2[1])
                    int2 = [x2, y2, z_plane]  # INTERSECTION 2
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

                    points = [tuple(v) for v in v_on]
                    for i in range(3):
                        edge = (points[i], points[(i + 1) % 3])
                        slice_plane.append(edge)
                    continue

                # Case 6: Facet does not intersect current z_plane
                else:
                    continue

                # Interpolate + Add to x/y values
                z_term1 = (z_plane - p1[2]) / (p3[2] - p1[2])
                x1 = p1[0] + z_term1 * (p3[0] - p1[0])
                y1 = p1[1] + z_term1 * (p3[1] - p1[1])
                int1 = [x1, y1, z_plane]  # INTERSECTION 1

                z_term2 = (z_plane - p2[2]) / (p3[2] - p2[2])
                x2 = p2[0] + z_term2 * (p3[0] - p2[0])
                y2 = p2[1] + z_term2 * (p3[1] - p2[1])
                int2 = [x2, y2, z_plane]  # INTERSECTION 2
                # Add intersections to slice plane intersection
                slice_plane.append((tuple(int1), tuple(int2)))

            # Add the current plane and the intersections for plane in the big list
            if z_plane == z_planes[-1] or z_plane == z_planes[0]:
                flats = set()
                for p1, p2 in slice_plane:
                    flats.add(p1)
                    flats.add(p2)

                flats = list(flats)
                flats_2d = np.array([[p[0], p[1]] for p in flats])

                if len(flats_2d) >= 3:
                    hull = ConvexHull(
                        flats_2d, qhull_options="QJ"
                    )  # QJ for quick hull, Pp for preserve point order
                    outer_contour = []
                    for idx in hull.vertices:
                        pt = list(flats_2d[idx]) + [z_plane]
                        outer_contour.append(tuple(np.round(pt, 6)))

                    for i in range(len(outer_contour)):
                        p1 = outer_contour[i]
                        p2 = outer_contour[(i + 1) % len(outer_contour)]
                        slice_plane.append((p1, p2))

            intersections.append([z_plane, slice_plane])

        ##### ------ Closed Contour Loops ------- #####

        contours = {}
        tolerance = tolerance

        # Loop through planes
        for plane in intersections:
            z_val = plane[0]  # Current z-plane
            seg_data = plane[1]  # List of segments (intersections)

            # Converto segments to numpy arrays
            segments = [(np.array(p1), np.array(p2)) for p1, p2 in seg_data]

            if z_val == z_planes[0] or z_val == z_planes[-1]:
                flat_points = []
                for p1, p2 in segments:
                    flat_points.extend([p1[:2], p2[:2]])

                flat_points = np.unique(np.round(flat_points, 6), axis=0)
                unique_pts = np.unique(flat_points, axis=0)

                if len(unique_pts) >= 3:
                    hull = ConvexHull(unique_pts, qhull_options="QJ")
                    loop = []
                    for idx in hull.vertices:
                        x, y = unique_pts[idx]
                        loop.append([x, y, z_val])
                    loop.append(loop[0])  # Close the loop explicitly
                    contours[z_val] = [loop]
                    continue  # skip rest of contour generation

            unique_segments = {}

            # Spatial grid size
            grid_size = max((x_max - x_min) / 100, (y_max - y_min) / 100, 1e-4)

            # Convert grid cell to list of segments
            spatial_grid = defaultdict(list)
            for seg in segments:
                # Rounding for precision
                p1 = tuple(np.round(seg[0], 6))
                p2 = tuple(np.round(seg[1], 6))
                # Sorting points (to avoid duplicates)
                sorted_seg = tuple(sorted([p1, p2]))

                # Spatial grid
                # Boolean for duplicates
                dup = False
                # Check both points
                for cell in [p1, p2]:
                    grid_key = (
                        int(cell[0] / grid_size),
                        int(cell[1] / grid_size),
                    )  # Get grid cell
                    # Check nearby segments in grid
                    for existing in spatial_grid.get(grid_key, []):
                        if np.allclose(
                            sorted_seg[0], existing[0], atol=tolerance
                        ) and np.allclose(sorted_seg[1], existing[1], atol=tolerance):
                            dup = True
                            break
                    if dup:
                        break
                # If not duplicate --> add to unique segments
                if not dup:
                    unique_segments[sorted_seg] = seg
                    # Add to spatial grid
                    spatial_grid[grid_key].append(sorted_seg)

            # Redefine segemtns list with unique segments
            segments = list(unique_segments.values())

            ## ------ Adjacency Map ------- ##
            # Build a graph of connections between points
            adj = defaultdict(list)

            for seg in segments:
                # Rounding for precision
                p1 = tuple(np.round(seg[0], 6))
                p2 = tuple(np.round(seg[1], 6))
                #
                adj[p1].append(p2)
                adj[p2].append(p1)

            # Track visited nodes
            visited = set()
            # List for closed contour loops
            loops = []

            # Adjacency map loop
            for start in adj:
                if start in visited:
                    # Skip point if visited already
                    continue

                contour = []
                curr = start

                while True:
                    # Add current point to contour path
                    contour.append(curr)
                    # Mark current point as visited
                    visited.add(curr)
                    # Get neighbors of current point (from adj list)
                    neighbors = [p for p in adj[curr] if p not in visited]

                    # If no neighbors - check if we are back to start
                    if not neighbors:
                        if len(contour) > 2 and np.allclose(
                            contour[0], curr, atol=tolerance
                        ):
                            contour.append(
                                contour[0]
                            )  # Append start point to close loop if ony 2 points
                        break
                    # Go to next point and start loop again
                    curr = neighbors[0]

                # ----- Clean and Finalize Contour ----- #
                if len(contour) > 3:
                    # If loop already closed, remove duplicate point
                    if np.allclose(contour[0], contour[-1], atol=tolerance):
                        contour = contour[:-1]

                    clean_contour = []
                    prev = None
                    for p in contour:
                        p_rounded = tuple(np.round(p, 6))
                        # Add point if different from previous (to prevent duplicates)
                        if prev is None or not np.allclose(
                            p_rounded, prev, atol=tolerance
                        ):
                            clean_contour.append([p_rounded[0], p_rounded[1], z_val])
                            prev = p_rounded

                    # Add cleaned contours to this loop
                    loops.append(clean_contour)

            # Store loops in contours dict
            contours[z_val] = loops

        ##### ------ Infill Generation ------- #####

        # Convert infill angle to radians for rotation matrix
        theta = np.radians(infill_angle)

        # Construct 2D rotation matrix
        rot = np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )
        # Transpose of rotation matrix
        rot_T = rot.T

        # Paths list for infill - INFILL PATHS LIST
        infill_paths = []

        # Iterates through each layer height and contours for layer height
        for layer_idx, (z_val, layer_contours) in enumerate(
            sorted(contours.items(), key=lambda x: x[0])
        ):
            # Skip if no contours for layer
            if not layer_contours:
                continue

            # Extracts X/Y coordinates of contours --> 2D plane
            all_contours = [
                np.array(c[:-1])[:, :2] if len(c) > 3 else np.array(c)[:, :2]
                for c in layer_contours
            ]

            # Rotate contours
            rotated_contours = [contour @ rot for contour in all_contours]

            # Stack all rotated contours for bounding box in y
            all_points = np.vstack(rotated_contours)
            # Get y bounds
            y_min, y_max = all_points[:, 1].min(), all_points[:, 1].max()
            # x_min, x_max = all_points[:,0].min(), all_points[:,0].max()

            # Calculate spacing between infills based on provided density
            infill_space = nozzle_dia / infill_density

            # Create array of y lines for infill
            y_lines = np.arange(
                y_min - infill_space, y_max + infill_space, infill_space
            )

            # List for infill for each layer
            layer_infill = []

            # Loop through each y line
            for y_idx, y in enumerate(y_lines):
                inters = []
                nozzle_half = nozzle_dia / 2

                # Check intersections with rotated contours
                for rot_contour in rotated_contours:

                    # For each line in the y_lines check for intersections with the rotated contours
                    for i in range(len(rot_contour)):
                        p1 = rot_contour[i]
                        p2 = rot_contour[(i + 1) % len(rot_contour)]

                        # Skip vertical edges
                        if abs(p1[1] - p2[1]) < 1e-6:
                            continue

                        # If line crosses the current y line
                        if (p1[1] < y < p2[1]) or (p2[1] < y < p1[1]):
                            # Interpolate intersection point
                            t = (y - p1[1]) / (p2[1] - p1[1])
                            x_int = p1[0] + t * (p2[0] - p1[0])
                            inters.append(x_int)

                # Creating segments

                # Sort the intersections
                inters.sort()
                paired = []
                i = 0

                reverse_this_line = y_idx % 2 == 1  # Reverse every other line

                # Creating line segments from the intersection pairs

                # Fill in with pairs
                while i < len(inters):
                    if i + 1 >= len(inters):
                        break

                    if reverse_this_line:
                        paired.append((inters[i + 1], inters[i]))
                    else:
                        paired.append((inters[i], inters[i + 1]))
                    i += 2

                # Transform back to normal orientation
                for start_x, end_x in paired:

                    start_rot = np.array([start_x, y])
                    end_rot = np.array([end_x, y])

                    start_og = start_rot @ rot_T
                    end_og = end_rot @ rot_T

                    start_vec = np.array([start_og[0], start_og[1]])
                    end_vec = np.array([end_og[0], end_og[1]])

                    segment_vector = end_vec - start_vec
                    segment_length = np.linalg.norm(segment_vector)

                    if segment_length > nozzle_dia:
                        if infill_angle % 90 == 0:
                            # If infill angle is 0, 90, 180, or 270 degrees
                            direct = segment_vector / segment_length
                        else:
                            # For other angles, use the unit vector
                            off_sign_y = np.sign(segment_vector[1])
                            off_sign_x = np.sign(segment_vector[0])
                            direct = np.array([off_sign_x, off_sign_y])

                        start_off = start_vec + direct * nozzle_half
                        end_off = end_vec - direct * nozzle_half

                        start_off = np.round(start_off, 6)
                        end_off = np.round(end_off, 6)

                        layer_infill.append(
                            [
                                [start_off[0], start_off[1], z_val],
                                [end_off[0], end_off[1], z_val],
                            ]
                        )

                    else:
                        continue

            # Append the layer infill paths to the main paths list
            infill_paths.append(layer_infill)

        ##### ------ Data Manipulation ------- #####

        # Current data structure consists of 2 primary distinct outputs for the robot
        # 1. Contours
        # Contours is a dict with z_height as key values
        # Each value is a list of list of (x,y,z) points, where each outer list is a complete closed contour
        # So far we have only encountered 1 closed contour per layer

        # 2. Infill Paths
        # Infill_paths is a list of list of lists
        # Each outer list is a layer
        # Each inner list is a line that has 2 points (x,y,z)
        # These points are also in a list themselves

        # Final data goal is to have a list of lists
        # Each outer list will be a layer
        # Each inner list will have points (x,y,z) [once again as lists]
        #   that correspond to the combined contours and infill paths for that layer
        #   with contours first and then infill paths

        data_points = []

        z_offset = z_planes[0]

        for z_val in sorted(contours.keys()):
            layer_data = []

            if z_val in contours:
                for contour in contours[z_val]:

                    if len(contour) >= 2:
                        contour = contour[:-1]  # Remove closing point

                        p_first = np.array(contour[0][:2])
                        p_last = np.array(contour[-1][:2])

                        dx, dy = p_first - p_last

                        # Determine offset axis
                        if abs(dx) > abs(dy):
                            offset_point = p_first - np.array([np.sign(dx), 0]) * (
                                nozzle_dia / 2
                            )
                        elif abs(dy) > abs(dx):
                            offset_point = p_first - np.array([0, np.sign(dy)]) * (
                                nozzle_dia / 2
                            )
                        else:
                            offset_point = p_first - np.sign([dx, dy]) * (
                                nozzle_dia / 2
                            )

                        offset_3d = [
                            round(offset_point[0], 6),
                            round(offset_point[1], 6),
                            z_val,
                        ]
                        contour.append(offset_3d)

                    converted_contour = [
                        [
                            round(p[0] / 1000, 6),
                            round(p[1] / 1000, 6),
                            round((p[2] - z_offset) / 1000, 6),
                        ]
                        for p in contour
                    ]

                    layer_data.extend(converted_contour)

            layer_idx = list(contours.keys()).index(z_val)
            if layer_idx < len(infill_paths):
                for segment in infill_paths[layer_idx]:
                    for pt in segment:
                        layer_data.append(
                            [
                                round(pt[0] / 1000, 6),
                                round(pt[1] / 1000, 6),
                                round((pt[2] - z_offset) / 1000, 6),
                            ]
                        )

            data_points.append(layer_data)

        return data_points

        # pass


def main():
    """Fill in comment"""
    # Write any testing code here, will be executed if the file is ran directly. Usually the file will be used by importing class

    filepath = "/Users/adityarao/Desktop/Slicing Algorithm/STL Files/Cube_STL_Simple.STL"  # Opening STL and reading as mesh
    slicing = STL2Motion(
        filepath, quaternion=None
    )  # Instantiate the class with the STL file and quaternion
    robot_points = (
        slicing.generate_robot_points()
    )  # Generate the robot points from the STL file
    # pass

    pprint.pprint(robot_points)  # Print the robot points to check the output


if __name__ == "__main__":
    main()
