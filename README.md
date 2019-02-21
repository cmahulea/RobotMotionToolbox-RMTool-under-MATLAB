# Robot Motion Toolbox
Robot Motion Toolbox for MATLAB

Robot Motion Toolbox (RMTool) offers a collection of tools devoted to modeling, path planning and motion control of mobile robots. RMTool is embedded in the MATLAB environment which provides the considerable advantage of creating powerful algebraic, statistical and graphical instruments exploiting the high quality routines available in MATLAB. It can be used for teaching mobile robotics in introductory courses since the user can interact with the tool and no previous knowledge of Matlab is required. The main features of the software package are:

* The environment map can be easily defined by the user by using the mouse
* Two kind of different (geometric) maps of the environment can be used:
    * Cell decomposition (discriminate between free and occupied cells). In particular, rectangular, triangular, polytopal, and trapezoidal decompositions are employed.
    * Road map (a set of routes within the free space). Visibility graph and generalized Voronoi diagram can be used.
    * Different methods for computing intermediate points when path planning is based on cell decomposition.
    * Different visualization variables (control actions and outputs).
    * Two motion control algorithms can be used: pure pursuit and a particular Proportional Integral (PI) controller.
    * Two kinematic robot models can be used to simulate the motion of a car-like robot and a differential-drive robot.
