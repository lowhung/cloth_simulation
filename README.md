
-----------------------
 COMPILING CPSC426
-----------------------

Below are quick instructions for compiling with default options. 

As with any graphics library you will need to make sure you already have the
opengl libraries on you computer. For example on Ubuntu 14.04 you will want
to install
```
freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev libglew1.6-dev mesa-utils libglew-dev premake4 libxrandr-dev
```
This will install opengl, glew and freegult.  

On Windows you will need to download freeglut.lib and the opengl header files and put them in your library path, tyically C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\lib\amd64 for freeglut.lib and C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\include for the GL folder.
Note: The build system uses [**premake4**](https://premake.github.io/download.html). You are going to need this available on your system in order to generate the build files. It has already been included in these files so you shouldn't need to download them.

### Windows 7/8/10 with Visual Studio 2012/2013/2015:
  1. Run generate_visual_studio.bat
  2. CPSC426.sln shoud be generated in vs2012
  3. All components should compile successfully.

### Linux/Unix:  

  1. ```./premake4_linux gmake```
      - make sure you are in the ./ directory.
  2. ```make config=debug64```
  	or
  	```make config=release64```
      - Depending on your preference for the type of build you want to perform you can build a debug version or a release version.
  4. All components are copied into the [x64|x32]/[Debug|Release]/ and lib/ directories.
  
     
### Mac OS X:
  For now, the process is similar to Linux/Unix.  
  
  1. ```./premake4_macos gmake```
      - make sure you are in the ./ directory.
  2. ```make config=debug64```
  	or
  	```make config=release64```
      - Depending on your preference for the type of build you want to perform you can build a debug version or a release version.
  4. All components are copied into the [x64|x32]/[Debug|Release]/ and lib/ directories.
  
  
  With OS X version
  10.4 or earlier, you may need to use an LD_LIBRARY_PATH environment
  variable for the executable to properly link with shared and dynamic
  libraries.
  
  -----------------
  Running the CODE
  -----------------

### Windows  
You can run the code from inside Visual Studio or you can run it an MS-DOS prompt fromt he root directory of the project  
 Example:  
 ```
 .\ass1
 ```
 
 
### Linux/Unix/Mac
You should run the code from ther terminal in the root directory of the project.  
 Example:  
 ```
 ./x64/Debug/CPSC426
 ```

----------------------
 USER INTERFACE
----------------------
Right mouse button drag rotates the camera

Mouse wheel zooms in/out

The scene control GUI panel has options to change the simulation timestep, cloth stiffness, wind strength, and control playback. The animation can be paused and stepped one frame at a time.


----------------------
 CODING CONVENTION
----------------------

cVar: class names are prefixed with 'c'
eVar: enum definitions are prefixed with 'e'
gVar: global variables are prefixed with 'g'
mVar: member variables are prefixed with 'm'

-----------------------------------
 Finding Your Way Around the Code
-----------------------------------

Libraries:

Eigen - commonly used linear algebra library. Tutorial (http://eigen.tuxfamily.org/dox/GettingStarted.html)

NanoGUI - A lightweight cross platform interface to be able to add gui elements to the OpenGL windows. Alows for interaction with the items in the simulation.

JsonCPP - library to parse json files in C++. In this assignment, it is used to parse files that specify parametric curves and other configurations


Code:
Files that need to be modified for the assignment are marked with  (*)

App.cpp
	- launches NanoGUI and runs the main program loop
	
Curve.cpp (*)
	- code for loading and evaluating a parametric curve
	- the control points are specified by text files using the JSON format
	- the parameter files for the curves are located in data/char_params and data/curve_params

scenarios/ClothScenario.cpp
	- animates a piece of cloth blowing in the wind

Shaders
	- vertex and pixel shaders can be found in data/shaders


----------------------
 CONTROL FLOW
----------------------
	- main() spawns a nanogui app (cApp) and initializes the mainloop.
	- every iteration of the app mainloop calls cApp::drawContents() which then calls Update() and DrawScenario()
	- cApp::Update() calls cApp::UpdateScenario(double time) to advance the scenario by one timestep
	- the scenario's Update(double time_elapsed) method is called every step to update the scene
	- in cClothScenario, Simulate() is called every update to step the simulation forward in time

----------------------
 DIRECTORY STRUCTURE
----------------------

The directory structure of this package is as follows:

  - data/          - A collection of premade json files that contain keyframe information for splines and an articulated figure.
  - include/       - external dependencies that are (legally) included
                  for convenience, but NOT part of THIS project.
  - jsoncpp/    - jsoncpp source code.
  - lib(s)/ 	- Where dynamic libraries are and compiled to for this project.
  - nanogui/    - nanogui source code.
  - render/     - source code pertaining to the rendering of the application.
  - scenarios/  - Source code that configures and controls the different "scenes" that can be used in this project. You can add or extend these.
  - util/ - Extra source code for common helper function, usually related to math.


----------------------
 EIGEN DATA STRUCTURES
----------------------

Eigen's basic data structures are vectors and matrices. These data stuctures can have different sizes 
and data types, the naming conventions are as follows:

Vector<Size><Type>: a vector of length <Size> containing data of <Type>.

Examples: 

	Vector4d is a vector containing 4 doubles

	Vector2f is a vector containing 2 floats
	
	Vector3i is a vector containing 3 integers
	
	VectorXd is a variable length vector containing doubles
	
	Matrix<Size><Type>: a square matrix that has <Size> rows and <Size> columns, containing data of <Type>

	Eigen::SparseMatrix is a sparse matrix

	Eigen::SparseLU is a solver for sparse matrices

Examples: 

	Matrix4d is a 4x4 matrix of doubles

	MatrixXf is a variable size matrix (doesn't have to be square) containing floats
