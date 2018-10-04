#Lightweight C++ implementation of the Generalized Label Correcting Method

##Overview
The implementation of the algorithm is in the .h files contained in the /include directory. The algorithm relies on virtual method provided by the user for a particular environment and system. There are two simple examples demonstrating the API to the algorithm. These demos use a single thread to compute the solution to aid in readability of the code. Multi-threaded implementations and GPU-acceleration of the user provided methods are necessary for real-time autonomous driving. 

##Examples 
The first example is implemented in shortest_path.cpp. It illustrates a single motion planning query in a simple environment with two circular obstacles. The planner is seeking to find a shortest path from the initial location (0,0) to a goal region at (10,10). The first example demonstrates standard kinematic planning in 2-dimensions. That is, there are no differential constraints.

The second example imposes a curvature constraint on the resulting path. The solution is visually different because the differential constraint limiting the curvature of the path prevents weaving between the obstacles and the solution is to go around the obstacles, taking a longer path.

More complex motion planning examples can be found in the accompanying documents and at 
https://www.youtube.com/watch?v=4-r6Oi8GHxc

##Compatibility with formal specifications
The algorithm is essentially constructing a sparse finite abstraction of the continuous model. Thus, by taking a synchronous product of the search tree with an algorithmically generated runtime monitor, the algorithm can be applied to complex task specifications expressed, for example, in linear temporal logic (LTL). This is discussed in the acompanying PhD thesis defense recording.

##Running the demos
This package will come with executable binaries that will run on a 64-bit machine running linux. However, the source code can be recompiled as follows:

mkdir build
cd build
cmake ..
make

The executables will be saved in the /examples directory. To run them enter

./shortest-path

or 

./car-demo

##Viewing output
The result of running one of these executables is saved to .txt files in the /plots directory. To view them, run the shortest_path_viewer.py python script. This will generate a .png file of the sample motion planning query. 