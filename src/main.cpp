#include "../include/primitives.h"

#include <iostream>

int main()
{

    Point a(0., 0.), b(1., 1.), c(.5, .5), d(2., 2.);

//    kdtree::PointSet st = kdtree::PointSet("test/etc/test2.dat");
    kdtree::PointSet points = kdtree::PointSet();

    points.put(a);
    points.put(b);
    points.put(c);
    points.put(d);

    std::cout << points.size() << '\n';
    const kdtree::PointSet pp = points;
    auto n = pp.nearest(Point(.4, .4));
    std::cout << *n << c << '\n';
    return 0;
}
