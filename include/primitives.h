#pragma once

#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

class Point
{
public:
    Point(double x, double y);

    double x() const
    {
        return p_x;
    }

    double y() const
    {
        return p_y;
    }

    double distance(const Point & point) const;

    bool operator<(const Point & point) const
    {
        return p_x < point.x() || (p_x == point.x() && p_y < point.y());
    }

    bool operator==(const Point & point) const
    {
        return std::fabs(p_x - point.x()) < std::numeric_limits<double>::epsilon() &&
                std::fabs(p_y - point.y()) < std::numeric_limits<double>::epsilon();
    }

    bool operator!=(const Point & point) const
    {
        return !(point == *this);
    }

    bool operator>(const Point & point) const
    {
        return point < *this;
    }

    bool operator<=(const Point & point) const
    {
        return !(point > *this);
    }

    bool operator>=(const Point & point) const
    {
        return point <= *this;
    }

    friend std::ostream & operator<<(std::ostream &, const Point &);

private:
    double p_x = 0.0, p_y = 0.0;
};

class Rect
{
public:
    Rect(const Point & left_bottom, const Point & right_top);
    double xmin() const
    {
        return left_bottom.x();
    }

    double ymin() const
    {
        return left_bottom.y();
    }

    double xmax() const
    {
        return right_top.x();
    }

    double ymax() const
    {
        return right_top.y();
    }

    double distance(const Point & p) const;

    bool contains(const Point & p) const
    {
        return (p.x() <= this->xmax() && p.x() >= this->xmin() && p.y() >= this->ymin() && p.y() <= this->ymax());
    }
    bool intersects(const Rect &) const;

private:
    const Point left_bottom;
    const Point right_top;
};

namespace rbtree {

class PointSet
{
public:
    class iterator
    {

    private:
        std::shared_ptr<std::set<Point>> st;
        std::set<Point>::iterator it;

    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;

        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;

        iterator() = default;

        iterator(const std::shared_ptr<std::set<Point>> & st, std::set<Point>::iterator it_set)
            : st(st)
            , it(it_set)
        {
        }

        iterator(std::set<Point>::iterator it_set)
            : it(it_set)
        {
        }

        friend bool operator==(const iterator & lhs, const iterator & rhs)
        {
            return lhs.it == rhs.it;
        }

        friend bool operator!=(const iterator & lhs, const iterator & rhs)
        {
            return lhs.it != rhs.it;
        }

        reference operator*() const
        {
            return (*it);
        }

        pointer operator->() const
        {
            return (it.operator->());
        }

        iterator & operator++()
        {
            ++it;
            return *this;
        }

        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }
    };

    PointSet(const std::string & filename = {});

    bool empty() const
    {
        return set_of_points.empty();
    }
    std::size_t size() const
    {
        return set_of_points.size();
    }
    void put(const Point & point)
    {
        set_of_points.insert(point);
    }
    bool contains(const Point & point) const
    {
        return set_of_points.find(point) != set_of_points.end();
    }

    // second iterator points to an element out of range
    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const
    {
        return iterator(set_of_points.begin());
    }

    iterator end() const
    {
        return iterator(set_of_points.end());
    }

    std::optional<Point> nearest(const Point &) const;
    // second iterator points to an element out of range
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

    std::set<Point> set_of_points;
};

} // namespace rbtree

namespace kdtree {

struct Node
{
    const Point point;

    std::size_t depth;

    std::shared_ptr<Node> left;
    std::shared_ptr<Node> right;

    Node(const Point point, std::size_t depth)
        : point(point)
        , depth(depth)
    {
    }
};

class PointSet
{
public:
    class iterator
    {
    private:
        std::vector<Point> points;

    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;

        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;

        iterator() = default;

        iterator(std::vector<Point> st)
            : points(std::move(st))
        {
        }

        friend bool operator==(const iterator & lhs, const iterator & rhs)
        {
            return lhs.points == rhs.points;
        }

        friend bool operator!=(const iterator & lhs, const iterator & rhs)
        {
            return !(lhs == rhs);
        }

        reference operator*() const
        {
            return points.back();
        }

        pointer operator->() const
        {
            return &points.back();
        }

        iterator & operator++()
        {
            points.pop_back();
            return *this;
        }

        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }
    };

    PointSet(const std::string & filename = {});

    PointSet(const PointSet & other)
    {
        for (const auto & x : other) {
            put(x);
        }
    }

    ~PointSet()
    {
        root.reset();
    }

    bool empty() const
    {
        return sz == 0;
    }

    std::size_t size() const
    {
        return sz;
    }

    void put(const Point &);

    bool contains(const Point & point) const
    {
        return find(root, point) != nullptr;
    }

    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const;

    iterator end() const
    {
        return {};
    }

    std::optional<Point> nearest(const Point &) const;
    std::pair<iterator, iterator> nearest(const Point &, std::size_t) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

    std::shared_ptr<Node> root;

private:
    std::size_t sz = 0;

    void range(const std::shared_ptr<Node> & current, const Rect & rect, std::vector<Point> & answer) const;
    const std::shared_ptr<Node> & find(const std::shared_ptr<Node> & current, const Point & point) const;
    void check_to_put_node(const std::shared_ptr<Node> & node, const Point & point);
    bool put_node(const std::shared_ptr<Node> & node, const Point & point, bool is_left);
    void dfs(const std::shared_ptr<kdtree::Node> & node, std::vector<Point> & answer) const;
    static bool comp_by_x(const Point & a, const Point & b);
    static bool comp_by_y(const Point & a, const Point & b);
    void create_2dtree(std::vector<Point> & points, size_t left, size_t right, size_t cnt);
};

} // namespace kdtree
