# lartkv5

## Description
The LAR toolkit is a software suite developed at LAR (http://lars.mec.ua.pt) since 2009 intended to support the software development involving robots, especially for the ATLAS project (http://atlas.web.ua.pt) and the PHUA project.
The initial versions (v1 and v2) used the CARMEN/IPC framework and later versions used ROS framework. Up to version 4, SVN was the version control support used, but with this version 5 (updated by 2014/2015) a migration to git was carried out.

The toolkit is quite a complex web of packages with many relations and interdependencies. A graphical view of these dependencies can be checked for version 4 (http://lars.mec.ua.pt/lartk4/) which is quite similar to this version 5.

## Using version v5
The lartkv5 was created using git in lar.mec.ua.pt server. The repository contains two folders, src and hardware.
Users from that repository must clone the repository

    git clone ssh: //USER@lars.mec.ua.pt/home/repositories/lar5 lar5

After cloned, it is necessary to start the ROS/catkin workspace inside the src folder, to create the main CMakeLists.txt (not under version control).

    cd src
    catkin_init_workspace

Next, you need to go up one folder and run catkin_make to create the build and devel folders along src and hardware folders. The build and devel folders must not be under version control.

    cd ..
    catkin_make

The repository compilation should work completely.
At first, it is possible to install some dependencies (**ros-indigo-sound-play**), but the packages with complicated manual install dependencies will not be compiled.
During the first phase of the compilation, warnings will appear about which packages will not be compiled.
Some packages will not compile because they issue some errors that I [Jorge Almeida] could not solve, in those cases I adjusted the `cmakelist` to avoid compilation attempts.
I sent to the professor the google spreadsheet file with the problems encountered in the migration.

## Updates by V. Santos, 08-Mar-2015,12:56
To enable the compilations of some modules, the following packages were needed:

* sudo apt-get install ros-indigo-sound-play
* sudo apt-get install libudev-dev
* sudo apt-get install libkml-dev              #detected by cmake
* sudo apt-get install libdmtx-dev             #detected by cmake
* sudo apt-get install gtkmm-3.0
* sudo apt-get install ros-indigo-gps-common
* sudo apt-get install libgraphviz-dev         #detected by cmake
* sudo apt-get install libcgal-dev             #detected by cmake

Also corrected several cases of:
 catkin_package () include dir 'include' does not exist relative to
   '/home/vitor/SWproj/lar5/src/sensors/odometer'
   Just edit the local CMakeLists.txt and comment a line of the kind:
 INCLUDE_DIRS include

in catkin_package specific configuration

## Further issues
It would be interesting to adopt a workflow based on branches and not centralized as it existed until now
    https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow
has a comparison of the various workflows and how they work in git, and can be useful.

New branches are simple to create:

    git checkout -b NEWBRANCH




