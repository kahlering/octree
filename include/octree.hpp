#pragma once

// Copyright (c) 2021 kahlering
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

/**
 * @brief The Octree class.
 *
 * @tparam IT Point cloud iterator
 * @tparam F Function object to make the coordinates of every Point type accessible to the octree. It takes IT::value_type
 *      as input and returns an array<double, 3> = {x, y, z}
 */
template <typename IT, typename F>
class Octree {
    using T = typename IT::value_type;

   public:
    /**
     * @brief Construct a new Octree object
     *
     * @param begin Iterator to the first point in the point cloud
     * @param end Iterator to the last point in the point cloud
     * @param bucket_size The bucket size determines the maximum number of points stored in the leafes
     * @param max_depth The maximum depth of the octree
     * @param get_coords Function object to make the coordinates of every Point type accessible to the octree. It should take IT::pointer
     *      as input and return a std::tuple<double, double, double> = {x, y, z}
     */
    Octree(const IT begin, const IT end, const size_t bucket_size, const size_t max_depth, F&& get_coords);

    // The copy constructor is diabled because the tree stores internal pointers. So a simple copy would most likely not have the expected behavior.
    Octree(const Octree& other) = delete;

    /**
     * @brief Finds and returns the points in the given area.
     *
     * @param query_point The center of the search
     * @param max_dist The radius of the search
     * @return std::vector<T*> A vector that contains all points in the search area
     */
    std::vector<typename IT::pointer> get_points_in_radius(const T* query_point, const double radius) const;

   private:
    struct Node {
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
        Node(double x, double y, double z, double size, typename std::vector<T*>::iterator begin, typename std::vector<T*>::iterator end);

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
        typename std::vector<T*>::iterator begin;
        // An iterator to the last point in the node
        typename std::vector<T*>::iterator end;
    };

    // defines the position of child nodes in their parent node.
    static constexpr std::array<int, 8> child_location_x = {-1, -1, -1, -1, 1, 1, 1, 1};
    static constexpr std::array<int, 8> child_location_y = {-1, -1, 1, 1, -1, -1, 1, 1};
    static constexpr std::array<int, 8> child_location_z = {-1, 1, -1, 1, -1, 1, -1, 1};

    // The bucket size of the octree.
    size_t bucket_size;
    // The maximum depth of the octree.
    size_t max_depth;
    // A vector that contains the point cloud.
    std::vector<T*> points;
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
     * @brief Calculates the coordinates of the center of the root node and necessary size of the root node from the input point cloud.
     *
     * @return std::array<double, 4>  The array has the following elements {x, y, z, octree_size}.
     */
    std::array<double, 4> get_root_node_pos_and_size();
    /**
     * @brief Finds and returns the points in the given area recursively. This function will be called by the public get_points_in_radius() function with
     *      the root node as starting point.
     *
     * @param query_point Center of the search.
     * @param max_dist The radius of the search.
     * @param n The node to search in.
     * @param points std::vector<T*> The matching points will be added to this vector.
     */
    void get_points_in_radius(const T* query_point, const double& max_dist, const Node& n, std::vector<T*>& points) const;
};



#include "octree.tpp"