#include <octree.hpp>

#include <iostream>
#include <vector>

// A simple Point class
struct Point {
    Point(double x, double y, double z) :x(x), y(y), z(z) {};
    Point() : x(0.0), y(0.0), z(0.0) {};
    double x;
    double y;
    double z;
};


int main()
{
    // create a small point cloud and fill it with sample points
	std::vector<Point> point_cloud;
    point_cloud.emplace_back(2.3, 2.0, 54.0);
    point_cloud.emplace_back(6.4, 5.7, 6.0);
    point_cloud.emplace_back(5.6, 24.1, 3.0);
    point_cloud.emplace_back(2.1, 2.0, 7.0);
    point_cloud.emplace_back(2.6, 2.0, 1.4);
    point_cloud.emplace_back(1.7, 2.0, 1.5);

    // Pass the begin and end iterator to the octree.
    // The bucket size in this example is 1.
    // The maximum depth is 10.
    // The last argument can be any function object that takes a Point as input and returns an array with the coordinates. In this case it is a lambda function.
    Octree tree(point_cloud.begin(), point_cloud.end(), 1, 10, [](const Point& p) {return std::array<double, 3>{p.x, p.y, p.z}; });
    
    // query the tree with the query point (2.0, 2.0, 2.0) and a search radius of 1.0
    std::vector<Point> result =  tree.get_points_in_radius(Point(2.0, 2.0, 2.0), 1.0);

    std::cout << "result" << std::endl;
    for (size_t i = 0; i < result.size(); ++i)
    {
        std::cout << i << ": " <<  result[i].x << " " << result[i].y << " " << result[i].z << std::endl;
    }
}


