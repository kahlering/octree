# octree

A simple templated Octree for radius neighbor search.


### Usage
The octree can be added to a project by either using *add_subdirectory(path/to/octree)* in CMake or by copying the *octree.hpp* header to the project.

There is a documented example in *src/example.cpp* that demonstrates how to use the octree with custom point types.

### Building the Tests and Examples

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Now, you can run the tests with

```
./tests
```

### Limitations
It is not possible to alter the point cloud after creating the octree without rebuilding it. if the point cloud changes, a new octree object has to be created.

The octree copies all points from the point cloud and rearanges them. For small point types this is faster than working with pointers but for large point types this means that a lot of data has to be moved around. 
