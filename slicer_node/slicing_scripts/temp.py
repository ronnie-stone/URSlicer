import numpy as np
from stl import mesh
import matplotlib.pyplot as plt

from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d .art3d import Poly3DCollection

import pprint

cube1 = mesh.Mesh.from_file('c:\\Users\\madhu\\OneDrive\\Desktop\\Slicer\\Test STLs\\Cube_STL_Simple.STL')
cube2 = mesh.Mesh.from_file('c:\\Users\\madhu\\OneDrive\\Desktop\\Slicer\\Test STLs\\Cube_STL_Complex.STL')

verts1 = cube1.vectors
normals1 = cube1.normals
verts2 = cube2.vectors
normals2 = cube2.normals

#v0 = cube.v0
#v1 = cube.v1
#v2 = cube.v2

print("Simple cube has:")
print("     ", len(normals1), "normal vectors")
print("     ", len(verts1), "facets")

print("Complex cube has:")
print("     ", len(normals2), "normal vectors")
print("     ", len(verts2), "facets")

#print(normals)
#print(verts)

facet_data = []
for i in range(len(cube1.vectors)):
    facet = [i, 
    cube1.v0[i].tolist(),
    cube1.v1[i].tolist(), 
    cube1.v2[i].tolist(),
    cube1.normals[i].tolist()]
    
    facet_data.append(facet)

#print(facet_data)
pprint.pprint(facet_data)