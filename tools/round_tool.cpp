#include <cada_shape.h>

using namespace cada::shape;

int main(int argc, char *argv[])
{
    // breve initialize
    // select two shapes
    auto &&s1 = ShapeFactory::instance()->createLine(Vec2d(1, 1), Vec2d(10, 4));
    auto &&s2 =
        ShapeFactory::instance()->createLine(Vec2d(1, 11), Vec2d(10, 6));

    bool trim = false;
    bool inverted = false;
    double radius = 10;

    // 13.15395011 6.9486833
    // 13.15395011 5.0513167
}