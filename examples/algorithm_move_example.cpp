#include <cada_shape.h>
#include <cada_ns.h>
#include <cada_math.h>
#include <iostream>
#include <iomanip>

using namespace cada::shape;

int main()
{
    // point
    {
        auto&& p1 = ShapeFactory::instance()->createPoint(Vec2d(1,1));
        p1->move(Vec2d(10, 0));
        std::cout << p1->to_string() << std::endl;
        p1->move(Vec2d(0, 10));
        std::cout << p1->to_string() << std::endl;
        p1->move(Vec2d(6, 6));
        std::cout << p1->to_string() << std::endl;
    }
    // line
    {
        auto &&l =
            ShapeFactory::instance()->createLine(Vec2d(0, 0), Vec2d(5, 5));
        l->move(Vec2d(2, 2));
        std::cout << l->to_string() << std::endl;
        l->move(Vec2d(4, 3));
        std::cout << l->to_string() << std::endl;
        l->move(Vec2d(-5, -3));
        std::cout << l->to_string() << std::endl;
    }
    // arc
    {
        auto &&a =
            ShapeFactory::instance()->createArc(Vec2d(0, 0), 5, 0, M_PI / 2);
        a->move(Vec2d(2, 2));
        std::cout << a->to_string() << std::endl;
        a->move(Vec2d(4, 3));
        std::cout << a->to_string() << std::endl;
        a->move(Vec2d(-5, -3));
        std::cout << a->to_string() << std::endl;
    }
    return 0;
}