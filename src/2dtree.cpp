#include "../include/primitives.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <optional>
#include <vector>

Point::Point(double x, double y)
    : p_x(x)
    , p_y(y)
{
}

double Point::distance(const Point & point) const
{
    double subtract_x = point.x() - p_x;
    double subtract_y = point.y() - p_y;
    return std::sqrt(subtract_x * subtract_x + subtract_y * subtract_y);
}

std::ostream & operator<<(std::ostream & out, const Point & point)
{
    out << point.x() << ' ' << point.y() << '\n';
    return out;
}

Rect::Rect(const Point & left_bottom, const Point & right_top)
    : left_bottom(left_bottom)
    , right_top(right_top)
{
}

double Rect::distance(const Point & p) const
{
    // when x_min = x_max (or y) the case is ambiguous
    if (p.x() <= this->xmax() && p.x() >= this->xmin() && p.y() >= this->ymin() && p.y() <= this->ymax()) {
        return 0.0;
    }
    if (p.x() <= this->xmax() && p.x() >= this->xmin()) {
        return std::min(std::abs(p.y() - this->ymin()), std::abs(p.y() - this->ymax()));
    }
    if (p.y() >= this->ymin() && p.y() <= this->ymax()) {
        return std::min(std::abs(p.x() - this->xmin()), std::abs(p.x() - this->xmax()));
    }
    return std::min(
            std::min(
                    std::sqrt((p.x() - this->xmax()) * (p.x() - this->xmax()) + (p.y() - this->ymax()) * (p.y() - this->ymax())),
                    std::sqrt((p.x() - this->xmin()) * (p.x() - this->xmin()) + (p.y() - this->ymin()) * (p.y() - this->ymin()))),
            std::min(
                    std::sqrt((p.x() - this->xmin()) * (p.x() - this->xmin()) + (p.y() - this->ymax()) * (p.y() - this->ymax())),
                    std::sqrt((p.x() - this->xmax()) * (p.x() - this->xmax()) + (p.y() - this->ymin()) * (p.y() - this->ymin()))));
}

bool Rect::intersects(const Rect & second) const
{
    return (second.xmax() <= this->xmax() && second.xmax() >= this->xmin() && second.ymax() >= this->ymin() && second.ymax() <= this->ymax()) ||
            (second.xmax() <= this->xmax() && second.xmax() >= this->xmin() && second.ymin() >= this->ymin() && second.ymin() <= this->ymax()) ||
            (second.xmin() <= this->xmax() && second.xmin() >= this->xmin() && second.ymax() >= this->ymin() && second.ymax() <= this->ymax()) ||
            (second.xmin() <= this->xmax() && second.xmin() >= this->xmin() && second.ymin() >= this->ymin() && second.ymin() <= this->ymax()) ||
            ((second.xmin() <= this->xmin() && second.xmax() >= this->xmin() && second.ymin() <= this->ymin() && second.ymax() >= this->ymin()) &&
             (second.xmin() <= this->xmax() && second.xmax() >= this->xmax() && second.ymin() <= this->ymax() && second.ymax() >= this->ymax())) ||
            ((second.xmin() >= this->xmin() && second.xmax() <= this->xmin() && second.ymin() >= this->ymin() && second.ymax() <= this->ymin()) &&
             (second.xmin() >= this->xmax() && second.xmax() <= this->xmax() && second.ymin() >= this->ymax() && second.ymax() <= this->ymax()));
}

std::ostream & operator<<(std::ostream & out, const rbtree::PointSet & setpoints)
{
    for (const auto & i : setpoints.set_of_points) {
        out << (i).x() << ' ' << (i).y() << '\n';
    }
    return out;
}

std::optional<Point> rbtree::PointSet::nearest(const Point & p) const
{
    auto answer = std::min_element(set_of_points.begin(), set_of_points.end(), [&](const Point & a, const Point & b) {
        return p.distance(a) < p.distance(b);
    });
    return *answer;
}

std::pair<rbtree::PointSet::iterator, rbtree::PointSet::iterator> rbtree::PointSet::nearest(const Point & p, std::size_t k) const
{
    auto near = std::make_shared<std::set<std::pair<double, Point>>>();
    auto near_poins = std::make_shared<std::set<Point>>();

    for (auto & i : set_of_points) {
        double dist = p.distance(i);
        near->emplace(dist, i);
        if (near->size() > k) {
            near->erase(--near->end());
        }
    }
    for (auto & i : *near) {
        near_poins->insert((i).second);
    }
    //    if (near.size() < k) {
    //        throw::std::invalid_argument("There are no " + std::to_string(k) + " nearest point(s)");
    //    }

    return {iterator(near_poins, near_poins->begin()), iterator(near_poins, near_poins->end())};
}

rbtree::PointSet::PointSet(const std::string & filename)
{
    std::ifstream in(filename);
    if (in.is_open()) {
        double x, y;
        while (in >> x) {
            in >> y;
            set_of_points.emplace(x, y);
        }
    }
    else if (!filename.empty()) {
        std::cerr << "File not found" << '\n';
    }
}

std::pair<rbtree::PointSet::iterator, rbtree::PointSet::iterator> rbtree::PointSet::range(const Rect & rect) const
{

    auto st = std::make_shared<std::set<Point>>();
    for (auto & i : set_of_points) {
        if (rect.contains(i)) {
            st->insert(i);
        }
    }
    //    if (near.size() < k) {
    //        throw::std::invalid_argument("There are no " + std::to_string(k) + "nearest point(s)");
    //    }

    return {iterator(st, st->begin()), iterator(st, st->end())};
}

///////////////////////////////////////////////             kdtree             /////////////////////////////////////////////////////////////////////////////

// kdtree::PointSet::PointSet(const std::string & filename)
//{
//     std::ifstream in(filename);
//     if (in.is_open()) {
//         double x, y;
//         while (in >> x) {
//             in >> y;
//             this->put({x, y});
//         }
//     }
//     else if (!filename.empty()) {
//         std::cerr << "File not found" << '\n';
//     }
// }

kdtree::PointSet::PointSet(const std::string & filename)
{
    std::ifstream in(filename);
    if (in.is_open()) {
        std::vector<Point> points;
        double x, y;
        while (in >> x) {
            in >> y;
            points.emplace_back(x, y);
        }
        create_2dtree(points, 0, points.size(), 0);
    }
    else if (!filename.empty()) {
        std::cerr << "File not found" << '\n';
    }
}

bool kdtree::PointSet::comp_by_x(const Point & a, const Point & b)
{
    return a.x() < b.x();
}

bool kdtree::PointSet::comp_by_y(const Point & a, const Point & b)
{
    return a.y() < b.y();
}

void kdtree::PointSet::create_2dtree(std::vector<Point> & points, std::size_t left, std::size_t right, std::size_t cnt)
{
    auto & sort_x = comp_by_x;
    auto & sort_y = comp_by_y;
    std::sort(points.begin() + left, points.begin() + right, cnt % 2 ? sort_x : sort_y);
    ++cnt;
    std::size_t mid = (right + left) / 2;
    put(points[mid]);

    if (right - left <= 1) {
        return;
    }

    create_2dtree(points, mid + 1, right, cnt);
    create_2dtree(points, left, mid, cnt);
}

const std::shared_ptr<kdtree::Node> & kdtree::PointSet::find(const std::shared_ptr<Node> & current, const Point & point) const
{
    if (current == nullptr || current->point == point) {
        return current;
    }
    if (current->depth % 2 == 0) {
        if (point.x() < current->point.x()) {
            if (current->point == point) {
                return current;
            }
            return find(current->left, point);
        }
        if (current->point == point) {
            return current;
        }
        return find(current->right, point);
    }
    if (point.y() < current->point.y()) {
        if (current->point == point) {
            return current;
        }
        return find(current->left, point);
    }
    if (current->point == point) {
        return current;
    }
    return find(current->right, point);
}

bool kdtree::PointSet::put_node(const std::shared_ptr<Node> & node, const Point & point, bool is_left)
{
    if (node->point.x() == point.x() && node->point.y() == point.y()) {
        return true;
    }

    if ((is_left ? node->left : node->right) == nullptr) {
        (is_left ? node->left : node->right) = std::make_shared<Node>(point, node->depth + 1);
        ++sz;
        return true;
    }
    check_to_put_node((is_left ? node->left : node->right), point);
    return false;
}

void kdtree::PointSet::check_to_put_node(const std::shared_ptr<Node> & node, const Point & point)
{
    if (node->depth % 2 == 0) {
        if (node->point.x() <= point.x()) {
            if (put_node(node, point, false)) {
                return;
            }
        }
        else if (node->point.x() >= point.x()) {
            if (put_node(node, point, true)) {
                return;
            }
        }
    }
    else {
        if (node->point.y() <= point.y()) {
            if (put_node(node, point, false)) {
                return;
            }
        }
        else if (node->point.y() >= point.y()) {
            if (put_node(node, point, true)) {
                return;
            }
        }
    }
}

void kdtree::PointSet::put(const Point & point)
{
    if (sz == 0) {
        root = std::make_shared<Node>(point, 0);
        sz++;
        assert(sz > 0);
        return;
    }
    check_to_put_node(root, point);

    return;
}

void kdtree::PointSet::dfs(const std::shared_ptr<Node> & node, std::vector<Point> & answer) const
{
    if (node == nullptr) {
        return;
    }
    answer.push_back(node->point);
    dfs(node->left, answer);
    dfs(node->right, answer);
}

kdtree::PointSet::iterator kdtree::PointSet::begin() const
{
    std::vector<Point> begin_v;
    dfs(root, begin_v);
    return iterator(begin_v);
}

std::pair<kdtree::PointSet::iterator, kdtree::PointSet::iterator> kdtree::PointSet::range(const Rect & rect) const
{
    std::vector<Point> points;
    range(root, rect, points);
    return {iterator(points), iterator()};
}

void kdtree::PointSet::range(const std::shared_ptr<Node> & current, const Rect & rect, std::vector<Point> & answer) const
{
    if (rect.contains(current->point)) {
        answer.push_back(current->point);
    }
    if (current->left != nullptr) {
        range(current->left, rect, answer);
    }
    if (current->right != nullptr) {
        range(current->right, rect, answer);
    }
}

std::pair<kdtree::PointSet::iterator, kdtree::PointSet::iterator> kdtree::PointSet::nearest(const Point & point, const std::size_t k) const
{
    std::set<std::pair<double, Point>> answer_points;
    std::vector<Point> answer;
    for (auto & i : *this) {
        answer_points.emplace(point.distance((i)), i);
        if (answer_points.size() > k) {
            answer_points.erase(--answer_points.end());
        }
    }
    answer.reserve(answer_points.size());
    for (auto & i : answer_points) {
        answer.push_back(i.second);
    }

    return {iterator(answer), iterator()};
}

std::optional<Point> kdtree::PointSet::nearest(const Point & point) const
{
    if (this->sz < 2) {
        return {};
    }
    auto answer = nearest(point, 1);

    return ((*answer.first));
}

std::ostream & operator<<(std::ostream & out, const kdtree::PointSet & setpoints)
{
    for (auto & i : setpoints) {
        out << ((i)).x() << ' ' << ((i)).y() << '\n';
    }
    return out;
}
