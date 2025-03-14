# ORC
WSU Capstone 2025

ORC is a active suspension system for an RC car, in this case RC4WD's Trail finder 2. The design documentation, source code, python code, custom parts, schematics, and PCBs are included in their own folders. 

The source code is intended to be built with Espressif's ESP-ID v5.4.0, which can be a standalone toolchain, or VScode extension. 
The python code is run with python 3.12 using the following libraries: csv, matplotlib, numpy, scipy, and collections. This will take the logged data, calculate some statistics, and plot the repeated runs.
The custom parts were designed in FreeCAD 1.0.0, with both STL files and the original files included. 
The schematics and PCBs were designed in KiCAD 8.0. A seperate "revisions" document details the shortcomings of this design. Fixes are not yet implemented to preserve the current state of the board and save development time.
