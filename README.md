# 2d-tree

### Данные
На двумерной евклидовой плоскости расположены N точек.

### Задача 1
Дан произвольный прямоугольник (x0, y0), (x1, y1). Из N точек необходимо выбрать все точки, которые попали внутрь этого прямоугольника.

### Задача 2
Дана произвольная точка А (xA, yA). Из N точек необходимо найти ближайшую к А точку.

![](https://www.cs.princeton.edu/courses/archive/fall19/cos226/assignments/kdtree/images/kdtree-ops.png)


Сложность методов ```PointSet::put(const Point &)``` и ```PointSet::contains(const Point &)``` О(logN)
Сложность методов ```PointSet::nearest(const Point &)``` и ```PointSet::range(const Rect &)``` O(N)


