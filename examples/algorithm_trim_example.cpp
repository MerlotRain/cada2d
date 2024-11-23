#include <cada_shape.h>
#include <iostream>
#include <iomanip>

using namespace cada::shape;

int main()
{
    // line
    {
        auto &&l =
            ShapeFactory::instance()->createLine(Vec2d(0, 0), Vec2d(10, 0));
        std::cout << l->to_string() << std::endl;
        l->trimStartPoint(Vec2d(5, 0), Vec2d(7, 0));
        std::cout << l->to_string() << std::endl;
    }
    return 0;
}