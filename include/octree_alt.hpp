#pragma once

//Copyright (c) 2021 kahlering
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

/**
 * @brief The Octree class.
 * 
 * @tparam IT Point cloud iterator
 * @tparam F Function object to make the coordinates of every Point type accessible to the octree. It takes IT::value_type
 *      as input and returns an array<double, 3> = {x, y, z}
 */
template<typename IT, typename F>
class Octree
{
    using T = typename IT::value_type;
public:
    
    /**
     * @brief Construct a new Octree object
     * 
     * @param begin Iterator to the first point in the point cloud 
     * @param end Iterator to the last point in the point cloud 
     * @param bucket_size The bucket size determines the maximum number of points stored in the leafes
     * @param max_depth The maximum depth of the octree
     * @param get_coords Function object to make the coordinates of every Point type accessible to the octree. It takes IT::value_type
     *      as input and returns an array<double, 3> = {x, y, z}
     */
    Octree(const IT begin, const IT end, const size_t bucket_size, const size_t max_depth, F&& get_coords);

    // The copy constructor is diabled because the tree stores internal pointers. So a simple copy would most likely not have the expected behavior.
    Octree(const Octree& other) = delete;

    /**
     * @brief Finds and returns the points in the given area.
     * 
     * @param query_point The center of the search
     * @param max_dist The radius of the search 
     * @return std::vector<T> A vector that contains all points in the search area
     */
    std::vector<T> get_points_in_radius(const T& query_point, const double radius) const;

private:

    struct Node
    {
        /**
         * @brief Construct a new Node object
         * 
         * @param x The x-coordinate of the node
         * @param y The y-coordinate of the node
         * @param z The z-coordinate of the node
         * @param size The size of the node
         * @param begin An iterator to the first point in the node
         * @param end An iterator to the last point in the node
         */
        Node(double x, double y, double z, double size, typename std::vector<T>::iterator begin, typename std::vector<T>::iterator end);

        // returns true if this node is a leaf, otherwise false
        bool is_leaf() const;
        // returns true if the child with index child_idx (0-7) exists;
        bool child_exists(int child_idx) const;
        // returns the size of the node
        double get_size() const;
        
        // The x-coordinate of the node
        double x;
        // The y-coordinate of the node
        double y;
        // The z-coordinate of the node
        double z;
        // The size of the node
        double size;
        // The indices of of the child nodes in the Octree::nodes vector
        std::array<size_t, 8> children;
        // An iterator to the first point in the node
        typename std::vector<T>::iterator begin;
        // An iterator to the last point in the node
        typename std::vector<T>::iterator end;
    };

    // defines the position of child nodes in their parent node. 
    static constexpr std::array<int, 8> child_location_x = {-1,-1,-1,-1,1,1,1,1};
    static constexpr std::array<int, 8> child_location_y = {-1,-1,1,1,-1,-1,1,1};
    static constexpr std::array<int, 8> child_location_z = {-1,1,-1,1,-1,1,-1,1};

    // The bucket size of the octree.
    size_t bucket_size;
    // The maximum depth of the octree.
    size_t max_depth;
    // A vector that contains the point cloud.
    std::vector<T> points;
    // A vector that contains the nodes of the octree
    std::vector<Node> nodes;
    // a Function object to access the coordinates of the points
    const F get_coords;

    /**
     * @brief Constructs the Octree recursively.
     * 
     * @param node_idx The index of the root node.
     * @param current_depth The current recursion depth.
     */
    void build_tree(const size_t node_idx, const size_t current_depth);
    /**
     * @brief Calculates the coordinates and dimensions of the octree from the input point cloud.
     * 
     * @return std::array<double, 4>  The array has the following elements {x, y, z, octree_size}.
     */
    std::array<double, 4> get_dimensions();
    /**
     * @brief Finds and returns the points in the given area recursively. This function will be called by the public get_points_in_radius() function with
     *      the root node as starting point.
     * 
     * @param query_point Center of the search.
     * @param max_dist The radius of the search.
     * @param n The node to search in.
     * @param points std::vector<T> The matching points will be added to this vector.
     */
    void get_points_in_radius(const T& query_point, const double& max_dist, const Node& n, std::vector<T>& points) const;
};


template<typename IT, typename F>
Octree<IT, F>::Octree(const IT begin, const IT end, const size_t bucket_size, const size_t max_depth, F&& get_coords): bucket_size(bucket_size), max_depth(max_depth), get_coords(get_coords), points(begin, end)
{
    std::array<double, 4> d = get_dimensions();
    nodes.emplace_back(d[0], d[1], d[2], d[3], points.begin(), points.end());
    build_tree(0, 0);
}


template<typename IT, typename F>
Octree<IT, F>::Node::Node(double x, double y, double z, double size, typename std::vector<T>::iterator begin, typename std::vector<T>::iterator end):
x(x), y(y), z(z), size(size), begin(begin), end(end),children{0}
{
};

template<typename IT, typename F>
bool Octree<IT, F>::Node::is_leaf() const
{
    return std::all_of(children.begin(), children.end(), [](size_t i){return i == 0;});
}

template<typename IT, typename F>
bool Octree<IT, F>::Node::child_exists(int child_idx) const 
{
    return children[child_idx] > 0;
}

template<typename IT, typename F>
double Octree<IT, F>::Node::get_size() const
{
    return size;
}

template<typename IT, typename F>
void Octree<IT, F>::build_tree(const size_t node_idx, const size_t current_depth)
{
    if(std::distance(nodes[node_idx].begin, nodes[node_idx].end) <= bucket_size || current_depth >= max_depth)
    {
        return;
    }
    const double new_size = nodes[node_idx].size / 2;
    const double x = nodes[node_idx].x;
    const double y = nodes[node_idx].y;
    const double z = nodes[node_idx].z;
    const auto begin = nodes[node_idx].begin;
    const auto end = nodes[node_idx].end;

    std::array<typename std::vector<T>::iterator, 9> its;
    its[0] = begin;
    its[8] = end;
    its[4] = std::partition(begin, end, [x, this](const T& v) { return get_coords(v)[0] < x; });
    its[2] = std::partition(begin, its[4], [y, this](const T& v) { return get_coords(v)[1] < y; });
    its[6] = std::partition(its[4], end, [y, this](const T& v) { return get_coords(v)[1] < y; });

    its[1] = std::partition(begin, its[2], [z, this](const T& v) { return get_coords(v)[2] < z; });
    its[3] = std::partition(its[2], its[4], [z, this](const T& v) { return get_coords(v)[2] < z; });
    its[5] = std::partition(its[4], its[6], [z, this](const T& v) { return get_coords(v)[2] < z; });
    its[7] = std::partition(its[6], end, [z, this](const T& v) { return get_coords(v)[2] < z; });

    for(size_t i = 0; i < 8; ++i)
    {
        if(std::distance(its[i], its[i+1]) > 0)
        {
            nodes.emplace_back(x + std::copysign(new_size, child_location_x[i]) , y + std::copysign(new_size, child_location_y[i]), z + std::copysign(new_size, child_location_z[i]), new_size, its[i], its[i+1]);
            nodes[node_idx].children[i] = nodes.size() - 1;
            build_tree(nodes[node_idx].children[i], current_depth + 1);
        }
    }
}

template<typename IT, typename F>
std::array<double, 4> Octree<IT, F>::get_dimensions()
{
    double max_x = -std::numeric_limits<double>::infinity();
    double min_x = +std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_y = +std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double min_z = +std::numeric_limits<double>::infinity();
    for(const T& p : points)
    {
        auto coords = get_coords(p);
        max_x = std::max(max_x, coords[0]);
        min_x = std::min(min_x, coords[0]);
        max_y = std::max(max_y, coords[1]);
        min_y = std::min(min_y, coords[1]);
        max_z = std::max(max_z, coords[2]);
        min_z = std::min(min_z, coords[2]);
    }
    double size = std::max(max_z - min_z, std::max(max_x - min_x, max_y - min_y)) / 2.0;
    double x = (max_x + min_x) / 2;
    double y = (max_y + min_y) / 2;
    double z = (max_z + min_z) / 2;
    return std::array<double, 4>{x, y, z, size};
}


template<typename IT, typename F>
std::vector<typename IT::value_type> Octree<IT, F>::get_points_in_radius(const T& query_point, const double max_dist) const
{
    std::vector<T> result;
    get_points_in_radius(query_point, max_dist, nodes[0], result);
    return result;
}

template<typename IT, typename F>
void Octree<IT, F>::get_points_in_radius(const T& query_point, const double& max_dist, const Node& n, std::vector<T>& points) const
{
    auto coords_query = get_coords(query_point);
    double distance_to_node = std::sqrt(std::pow(n.x - coords_query[0], 2) + std::pow(n.y - coords_query[1], 2) + std::pow(n.z - coords_query[2], 2));
    if (distance_to_node > (max_dist + n.get_size() * std::numbers::sqrt3))
    {
        return;
    }

    if (distance_to_node + n.get_size() * std::numbers::sqrt3 <= max_dist)
    {
        points.insert(points.end(), n.begin, n.end);
        return;
    }

    if (n.is_leaf())
    {
        for (typename std::vector<T>::const_iterator it = n.begin; it != n.end; it++)
        {
            auto coords_it = get_coords(*it);
            double distance_points = std::sqrt(std::pow(coords_it[0] - coords_query[0], 2) + std::pow(coords_it[1] - coords_query[1], 2) + std::pow(coords_it[2] - coords_query[2], 2));
            if( distance_points <= max_dist)
            {
                points.push_back(*it);
            }
        }
        return;
    }

    for (size_t i = 0; i < 8; i++)
    {
        if (n.child_exists(i))
        {
            get_points_in_radius(query_point, max_dist, nodes[n.children[i]], points);
        }
    }
}
