# SolveSpaceLib

An insolated version of the geometric constraint solver from SolveSpace: libslvs

See http://solvespace.com

The original library code is not independent of the complete SolveSpace cad application, 
which is somewhat inconvenient if all you need is the constraint solver.

This repository only contains the neccessary code, and is thus quite small. 
As far as I know, this is the only open source working geometrical constraint solver out there.

## Build instructions

Use CMake to create a build system, or IDE project files.

The library itself is within the folder libslvs, which has its own CMakeList.txt file
On top level the CMakeList.txt is only an example on how to use the library from an application.

