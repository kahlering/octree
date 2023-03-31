
template <typename IT, typename F>
Octree<IT, F>::Octree(const IT begin, const IT end, const size_t bucket_size, const size_t max_depth, F&& get_coords) : bucket_size(bucket_size), max_depth(max_depth), get_coords(get_coords) {
    for (auto it = begin; it != end; ++it) {
        points.emplace_back(&(*it));
    }
    std::array<double, 4> d = get_root_node_pos_and_size();
    nodes.emplace_back(d[0], d[1], d[2], d[3], points.begin(), points.end());
    build_tree(0, 0);
}

template <typename IT, typename F>
Octree<IT, F>::Node::Node(double x, double y, double z, double size, typename std::vector<T*>::iterator begin, typename std::vector<T*>::iterator end) : x(x), y(y), z(z), size(size), begin(begin), end(end), children{0} {};

template <typename IT, typename F>
bool Octree<IT, F>::Node::is_leaf() const {
    return std::all_of(children.begin(), children.end(), [](size_t i) { return i == 0; });
}

template <typename IT, typename F>
bool Octree<IT, F>::Node::child_exists(int child_idx) const {
    return children[child_idx] > 0;
}

template <typename IT, typename F>
double Octree<IT, F>::Node::get_size() const {
    return size;
}

template <typename IT, typename F>
void Octree<IT, F>::build_tree(const size_t node_idx, const size_t current_depth) {
    if (std::distance(nodes[node_idx].begin, nodes[node_idx].end) <= bucket_size || current_depth >= max_depth) {
        return;
    }
    const double new_size = nodes[node_idx].size / 2;
    const double x = nodes[node_idx].x;
    const double y = nodes[node_idx].y;
    const double z = nodes[node_idx].z;
    const auto begin = nodes[node_idx].begin;
    const auto end = nodes[node_idx].end;

    std::array<typename std::vector<T*>::iterator, 9> its;
    its[0] = begin;
    its[8] = end;
    its[4] = std::partition(begin, end, [x, this](const T* v) { return std::get<0>(get_coords(v)) < x; });
    its[2] = std::partition(begin, its[4], [y, this](const T* v) { return std::get<1>(get_coords(v)) < y; });
    its[6] = std::partition(its[4], end, [y, this](const T* v) { return std::get<1>(get_coords(v)) < y; });

    its[1] = std::partition(begin, its[2], [z, this](const T* v) { return std::get<2>(get_coords(v)) < z; });
    its[3] = std::partition(its[2], its[4], [z, this](const T* v) { return std::get<2>(get_coords(v)) < z; });
    its[5] = std::partition(its[4], its[6], [z, this](const T* v) { return std::get<2>(get_coords(v)) < z; });
    its[7] = std::partition(its[6], end, [z, this](const T* v) { return std::get<2>(get_coords(v)) < z; });

    for (size_t i = 0; i < 8; ++i) {
        if (std::distance(its[i], its[i + 1]) > 0) {
            nodes.emplace_back(x + std::copysign(new_size, child_location_x[i]), y + std::copysign(new_size, child_location_y[i]), z + std::copysign(new_size, child_location_z[i]), new_size, its[i], its[i + 1]);
            nodes[node_idx].children[i] = nodes.size() - 1;
            build_tree(nodes[node_idx].children[i], current_depth + 1);
        }
    }
}

template <typename IT, typename F>
std::array<double, 4> Octree<IT, F>::get_root_node_pos_and_size() {
    double max_x = -std::numeric_limits<double>::infinity();
    double min_x = +std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_y = +std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double min_z = +std::numeric_limits<double>::infinity();
    for (const T* p : points) {
        auto coords = get_coords(p);
        max_x = std::max(max_x, std::get<0>(coords));
        min_x = std::min(min_x, std::get<0>(coords));
        max_y = std::max(max_y, std::get<1>(coords));
        min_y = std::min(min_y, std::get<1>(coords));
        max_z = std::max(max_z, std::get<2>(coords));
        min_z = std::min(min_z, std::get<2>(coords));
    }
    double size = std::max(max_z - min_z, std::max(max_x - min_x, max_y - min_y)) / 2.0;
    double x = (max_x + min_x) / 2;
    double y = (max_y + min_y) / 2;
    double z = (max_z + min_z) / 2;
    return std::array<double, 4>{x, y, z, size};
}

template <typename IT, typename F>
std::vector<typename IT::pointer> Octree<IT, F>::get_points_in_radius(const T* query_point, const double max_dist) const {
    std::vector<T*> result;
    get_points_in_radius(query_point, max_dist, nodes[0], result);
    return result;
}

template <typename IT, typename F>
void Octree<IT, F>::get_points_in_radius(const T* query_point, const double& max_dist, const Node& n, std::vector<T*>& points) const {
    auto coords_query = get_coords(query_point);
    double distance_to_node = std::sqrt(std::pow(n.x - std::get<0>(coords_query), 2) + std::pow(n.y - std::get<1>(coords_query), 2) + std::pow(n.z - std::get<2>(coords_query), 2));
    if (distance_to_node > (max_dist + n.get_size() * std::numbers::sqrt3)) {
        return;
    }

    if (distance_to_node + n.get_size() * std::numbers::sqrt3 <= max_dist) {
        points.insert(points.end(), n.begin, n.end);
        return;
    }

    if (n.is_leaf()) {
        for (typename std::vector<T*>::const_iterator it = n.begin; it != n.end; it++) {
            auto coords_it = get_coords(*it);
            double distance_points = std::sqrt(std::pow(std::get<0>(coords_it) - std::get<0>(coords_query), 2) + std::pow(std::get<1>(coords_it) - std::get<1>(coords_query), 2) + std::pow(std::get<2>(coords_it) - std::get<2>(coords_query), 2));
            if (distance_points <= max_dist) {
                points.push_back(*it);
            }
        }
        return;
    }

    for (size_t i = 0; i < 8; i++) {
        if (n.child_exists(i)) {
            get_points_in_radius(query_point, max_dist, nodes[n.children[i]], points);
        }
    }
}
