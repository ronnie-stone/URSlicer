# Immediate Next Steps

These are a set of instructions, recommendations, and guidelines for the next steps for complete systems integration.

This guide is assuming that you have read the primary readme for this repo. 

---

## Current Testing Status

Currently all of the system has been tested in isolation and in partial combination. This is the status of each subsystem.

* ROS2 Management Nodes: Fully working with some minor integration things left to be finished

* Temperature Management: Fault detection needs to be tested

* Extruder: Motor speed fine tuning

* Slicing: Finished but outputs need to be double checked

We have performed testing with printing a line and going through the motions of simple STLs but due to temperature management and extrusion timing it hasn't been fully finished. 

---

## Relevant Information and Code Locations

* Temperature Management: Refer to Arduino code

* Extruder Speed: Refer to Arduino code

* Bed Placement: printer_manager_component.cpp, line 207:210

* Extrusion Timing: printer_manager_component.cpp, line 314:321

* Slicing Verification: slicer_node.py, line 39:42

* Object Placement: printer_manager_component.cpp, line 269:276

* Launch Files: Refer to each package for relevant launch files. The whole system launch files are contained within the printing_manager package.

