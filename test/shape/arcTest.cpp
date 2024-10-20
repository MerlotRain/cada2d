#include <cada_shape.h>
#include <iostream>

using namespace cada::shape;

int main(int argc, char **argv)
{
    Arc arc(Vec2d(0, 0), 5, 0, 90);
    std::cout << arc.getRadius();
    std::cout << arc.getStartAngle();
    std::cout << arc.getEndAngle();
    return 0;
}