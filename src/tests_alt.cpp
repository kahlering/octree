

#include "octree_alt.hpp"

#include <iostream>
#include <list>
#include <deque>
#include <random>
#include <numbers>
#include <array>
#include <vector>
#include <cstdint>
#include <chrono>




struct Point
{
    Point(double x, double y, double z) :x(x), y(y), z(z) {};
    Point(): x(0.0), y(0.0), z(0.0){};
    double x;
    double y;
    double z;
    int asd[124];
};

std::ostream& operator<<(std::ostream& os, const Point& p)
{
    os << p.x << " " << p.y << " " << p.z << " ";
    return os;
}


template<typename T>
void rotate_point_cloud_x(T begin, T end, double angle)
{
    for (T it = begin; it != end; ++it)
    {
       double y_tmp = std::cos(angle) * it->y - std::sin(angle) * it->z;
       double z_tmp = std::sin(angle) * it->y + std::cos(angle) * it->z;
       it->y = y_tmp;
       it->z = z_tmp;
    }
}

template<typename T>
void rotate_point_cloud_y(T begin, T end, double angle)
{
    for (T it = begin; it != end; ++it)
    {
        double x_tmp = std::cos(angle) * it->x + std::sin(angle) * it->z;
        double z_tmp = -std::sin(angle) * it->x + std::cos(angle) * it->z;
        it->x = x_tmp;
        it->z = z_tmp;
    }
}

template<typename T>
void rotate_point_cloud_z(T begin, T end, double angle)
{
    for (T it = begin; it != end; ++it)
    {
        double x_tmp = std::cos(angle) * it->x - std::sin(angle) * it->y;
        double y_tmp = std::sin(angle) * it->x + std::cos(angle) * it->y;
        it->x = x_tmp;
        it->y = y_tmp;
    }
}

template<typename T>
std::vector<Point> exhaustive_neighbor_search(T begin, T end, const Point& query_point, double max_dist){
    std::vector<Point> result;
    for (T it = begin; it != end; ++it)
    {
        double distance = sqrt(pow(query_point.x - it->x, 2) + pow(query_point.y - it->y, 2) + pow(query_point.z - it->z, 2));
        if(distance <= max_dist)
        {
            result.push_back(*it);
        }
    }
    return result;
}

bool compare_results(std::vector<Point>& v1, std::vector<Point>& v2)
{
    if(v1.size() != v2.size())
    {   
        std::cout << "results have different sizes: v1 size " << v1.size() << " v2 size " << v2.size() <<  std::endl;
        return false;
    }
    for(Point& p1 : v1)
    {
        auto it = std::find_if(v2.begin(), v2.end(), [&p1](Point& p2){return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;});
        if(it == v2.end())
        {
            return false;
        }
    }
    return true;
}

template<typename T>
T generate_random_point_cloud_cube(size_t num_points, int_fast64_t random_seed=0, double size=10.0, double max_translation=100000.0)
{
    std::mt19937_64 gen(random_seed);
    std::uniform_real_distribution<double> dis(-size, size);
    std::uniform_real_distribution<double> angle_dis(0.0, 2 * std::numbers::pi);
    std::uniform_real_distribution<double> translation_dis(-max_translation, max_translation);
    double translation_x = translation_dis(gen);
    double translation_y = translation_dis(gen);
    double translation_z = translation_dis(gen);
    T points;
    for(size_t i = 0; i < num_points; ++i)
    {
        Point p;
        p.x = dis(gen) + translation_x;
        p.y = dis(gen) + translation_y;
        p.z = dis(gen) + translation_z;
        points.push_back(p);
    }
    rotate_point_cloud_x(points.begin(), points.end(), angle_dis(gen));
    rotate_point_cloud_y(points.begin(), points.end(), angle_dis(gen));
    rotate_point_cloud_z(points.begin(), points.end(), angle_dis(gen));
    return points;
}

template<typename T>
T generate_random_point_cloud_cylinder(size_t num_points, int_fast64_t random_seed=0, double length = 10.0, double radius = 1.0, double max_translation=10.0)
{
    std::mt19937_64 gen(random_seed);
    std::uniform_real_distribution<double> length_dis(0.0, length);
    std::uniform_real_distribution<double> angle_dis(0.0, 2 * std::numbers::pi);
    std::uniform_real_distribution<double> translation_dis(-max_translation, max_translation);
    double translation_x = translation_dis(gen);
    double translation_y = translation_dis(gen);
    double translation_z = translation_dis(gen);
    T points;
    for (size_t i = 0; i < num_points; ++i)
    {
        Point p;
        p.x = length_dis(gen) + translation_x;
        p.y = radius * std::sin(angle_dis(gen)) + translation_y;
        p.z = radius * std::cos(angle_dis(gen)) + translation_z;
        points.push_back(p);
    }
    rotate_point_cloud_y(points.begin(), points.end(), angle_dis(gen));
    rotate_point_cloud_z(points.begin(), points.end(), angle_dis(gen));
    rotate_point_cloud_x(points.begin(), points.end(), angle_dis(gen));
    return points;
}




// template<typename T = std::vector<Point>>
// void test_tree_cube(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed) {
//     auto points = generate_random_point_cloud_cube<T>(num_points, random_seed);
//     Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](Point p) {return std::array<double, 3>{p.x, p.y, p.z}; });
//     auto r1 = tree.get_points_in_radius(points.front(), 1.0);
//     auto r2 = exhaustive_neighbor_search(points.begin(), points.end(), points.front(), 1.0);
//     if (compare_results(r1, r2))
//     {
//         std::cout << "cube test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points passed" << std::endl;
//     }
//     else
//     {
//         std::cout << "cube test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points failed" << std::endl;
//     }
// }

// template<typename T = std::vector<Point>>
// void test_tree_cube_full(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed) {
//     auto points = generate_random_point_cloud_cube<T>(num_points, random_seed);
//     Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](Point p) {return std::array<double, 3>{p.x, p.y, p.z}; });
//     for (auto it = points.begin(); it != points.end(); ++it)
//     {
//         auto r1 = tree.get_points_in_radius(*it, 1.0);
//         auto r2 = exhaustive_neighbor_search(points.begin(), points.end(), *it, 1.0);
//         if (!compare_results(r1, r2))
//         {
//             std::cout << "cube test full with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points failed" << std::endl;
//         }
//     }
//     std::cout << "cube test full with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points passed" << std::endl;
// }


// void test_tree_cylinder(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed) {
//     auto points = generate_random_point_cloud_cylinder<std::vector<Point>>(num_points, random_seed);
//     Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](Point p) {return std::array<double, 3>{p.x, p.y, p.z}; });
//     auto r1 = tree.get_points_in_radius(points.front(), 1.0);
//     auto r2 = exhaustive_neighbor_search(points.begin(), points.end(), points.front(), 1.0);
//     if (compare_results(r1, r2))
//     {
//         std::cout << "cylinder test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points passed" << std::endl;
//     }
//     else
//     {
//         std::cout << "cylinder test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points failed" << std::endl;
//     }
// }

// void test_outlier(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed)
// {
//     auto points = generate_random_point_cloud_cube<std::vector<Point>>(num_points, random_seed);
//     points.emplace_back(52.0, 567.0, 67.0);
//     points.emplace_back(-50.6, 43.0, 5.0);
//     points.emplace_back(0.0, 68.5, 100.3);
//     Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](Point p) {return std::array<double, 3>{p.x, p.y, p.z}; });
//     auto r1 = tree.get_points_in_radius(points.back(), 1.0);
//     auto r2 = exhaustive_neighbor_search(points.begin(), points.end(), points.back(), 1.0);
//     if (compare_results(r1, r2))
//     {
//         std::cout << "outlier test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points passed" << std::endl;
//     }
//     else
//     {
//         std::cout << "outlier test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points failed" << std::endl;
//     }
// }

// void test_no_neighbors(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed)
// {
//     auto points = generate_random_point_cloud_cube<std::vector<Point>>(num_points, random_seed);
//     Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](Point p) {return std::array<double, 3>{p.x, p.y, p.z}; });
//     auto r1 = tree.get_points_in_radius(Point(100, 100, 100), 1.0);
//     auto r2 = exhaustive_neighbor_search(points.begin(), points.end(), Point(100, 100, 100), 1.0);
//     if (compare_results(r1, r2))
//     {
//         std::cout << "no neighbors test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points passed" << std::endl;
//     }
//     else
//     {
//         std::cout << "no neighbors test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points failed" << std::endl;
//     }
// }

// void test_same_points_multiple_times(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed)
// {
//     auto points = generate_random_point_cloud_cube<std::vector<Point>>(num_points, random_seed);
//     for (size_t i = 0; i < 100; ++i)
//     {
//         points.emplace_back(5, 5, 5);
//     }
//     Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](Point p) {return std::array<double, 3>{p.x, p.y, p.z}; });
//     auto r1 = tree.get_points_in_radius(points.back(), 1.0);
//     auto r2 = exhaustive_neighbor_search(points.begin(), points.end(), points.back(), 1.0);
//     if (compare_results(r1, r2))
//     {
//         std::cout << "same point multiple times test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points passed" << std::endl;
//     }
//     else
//     {
//         std::cout << "same point multiple times test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points failed" << std::endl;
//     }
// }

// void test_multiple_cylinders(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed)
// {
//     std::vector<std::vector<Point>> cylinders;
//     cylinders.push_back(generate_random_point_cloud_cylinder<std::vector<Point>>(num_points / 5, random_seed + 0, 10.0, 1.0, 10.0));
//     cylinders.push_back(generate_random_point_cloud_cylinder<std::vector<Point>>(num_points / 5, random_seed + 1, 8.0, 1.0, 10.0));
//     cylinders.push_back(generate_random_point_cloud_cylinder<std::vector<Point>>(num_points / 5, random_seed + 2, 2.0, 0.1, 10.0));
//     cylinders.push_back(generate_random_point_cloud_cylinder<std::vector<Point>>(num_points / 5, random_seed + 3, 17.0, 3.0, 10.0));
//     cylinders.push_back(generate_random_point_cloud_cylinder<std::vector<Point>>(num_points / 5, random_seed + 4, 0.3, 1.0, 10.0));
//     std::vector<Point> all_points;
//     for (size_t i = 0; i < cylinders.size(); ++i)
//     {
//         all_points.insert(all_points.end(), cylinders[i].begin(), cylinders[i].end());
//     }
//     Octree tree(all_points.begin(), all_points.end(), bucket_size, max_depth, [](Point p) {return std::array<double, 3>{p.x, p.y, p.z}; });
//     auto r1 = tree.get_points_in_radius(all_points.back(), 1.0);
//     auto r2 = exhaustive_neighbor_search(all_points.begin(), all_points.end(), all_points.back(), 1.0);
//     if (compare_results(r1, r2))
//     {
//         std::cout << "multiple cylinders test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points passed" << std::endl;
//     }
//     else
//     {
//         std::cout << "multiple cylinders test with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points failed" << std::endl;
//     }
// }

void performance_test_cube(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed, double search_radius=1.0)
{
    auto points = generate_random_point_cloud_cube<std::vector<Point>>(num_points, random_seed);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](const Point& p){return std::array<double, 3>{p.x, p.y, p.z};});
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "it took = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us] to construct the octree with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << std::endl;

    begin = std::chrono::steady_clock::now();
    auto r = tree.get_points_in_radius(points[0], 1.0);
    end = std::chrono::steady_clock::now();
    std::cout << "it took = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us] to query the octree with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points with a random query point " << std::endl;
    std::cout << r.size() << " points were found in the search area" << std::endl;
}

// void performance_test_cylinder(size_t bucket_size, size_t max_depth, size_t num_points, int_fast64_t random_seed, double search_radius=1.0)
// {
//     auto points = generate_random_point_cloud_cylinder<std::vector<Point>>(num_points, random_seed);
//     std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//     Octree tree(points.begin(), points.end(), bucket_size, max_depth, [](const Point& p){return std::array<double, 3>{p.x, p.y, p.z};});
//     std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//     std::cout << "it took = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us] to construct the octree with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << std::endl;

//     begin = std::chrono::steady_clock::now();
//     auto r = tree.get_points_in_radius(points[0], 1.0);
//     end = std::chrono::steady_clock::now();
//     std::cout << "it took = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us] to query the octree with bucket size " << bucket_size << " and max depth " << max_depth << " and " << num_points << " points with a random query point " << std::endl;
//     std::cout << r.size() << " points were found in the search area" << std::endl;
// }




int main(){

    // test_tree_cube(1, 10, 100000, 1);
    // test_tree_cube(10, 10, 100000,2);
    // test_tree_cube(100, 3, 100000, 3);
    // test_tree_cube(500, 40, 100000, 4);
    // test_tree_cube(35, 10, 100000, 5);
    // test_tree_cube(17, 10, 100000, 6);
    // test_tree_cube(17, 10, 10, 7);
    // test_tree_cube(1000000, 10, 1000000, 8);
    // test_tree_cube<std::list<Point>>(212, 4, 1000, 9);
    // test_tree_cube<std::deque<Point>>(134, 10, 100000, 10);
    
    // test_tree_cylinder(1, 10, 1000000, 11);
    // test_tree_cylinder(32, 32, 1000000, 12);
    // test_tree_cylinder(14, 10, 1000000, 13);
    // test_tree_cylinder(167, 10, 1000000, 14);

    // test_same_points_multiple_times(1, 10, 100000, 15);
    // test_same_points_multiple_times(10, 10, 100000, 16);
    // test_same_points_multiple_times(54, 1, 100000, 17);

    // test_outlier(1, 10, 100000, 18);
    // test_outlier(16, 14, 100000, 19);
    // test_outlier(20, 4, 100000, 20);

    // test_no_neighbors(1, 10, 10000, 21);
    // test_no_neighbors(10, 5, 10000, 22);
    // test_no_neighbors(20, 20, 10000, 23);

    // test_multiple_cylinders(1, 10, 100000, 24);
    // test_multiple_cylinders(10, 10, 100000, 25);
    // test_multiple_cylinders(100, 100, 100000, 26);
    // test_multiple_cylinders(1, 16, 100000, 27);

    // test_tree_cube_full(64, 10, 10000, 28);

    performance_test_cube(64, 10, 1000000, 29);
    //performance_test_cylinder(64, 10, 1000000, 30);

    return 0;
}