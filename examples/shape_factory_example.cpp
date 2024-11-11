#include <cada_shape.h>
#include <iostream>
#include <assert.h>

using namespace cada::shape;

void print_arc_information(const cada::shape::Arc *arc)
{
    assert(arc);
    std::cout << arc->to_string() << std::endl;
    std::cout << "     start point: " << arc->getStartPoint().to_string()
              << std::endl;
    std::cout << "     middle point: " << arc->getMiddlePoint().to_string()
              << std::endl;
    std::cout << "     end point: " << arc->getEndPoint().to_string()
              << std::endl;
}

int main(int argc, char **argv)
{
    // point
    {
        auto &&p1 = ShapeFactory::instance()->createPoint();
        std::cout << p1->to_string() << std::endl;

        auto &&p2 = ShapeFactory::instance()->createPoint(Vec2d(10.0, 12.0));
        std::cout << p2->to_string() << std::endl;

        auto &&p3 = ShapeFactory::instance()->createPoint(8.0, DBL_EPSILON);
        std::cout << p3->to_string() << std::endl;
    }
    // line
    {
        auto &&l1 = ShapeFactory::instance()->createLine();
        std::cout << l1->to_string() << std::endl;

        auto &&l2 = ShapeFactory::instance()->createLine(1.0, 2.0, 3.0, 4.0);
        std::cout << l2->to_string() << std::endl;

        auto &&l3 =
            ShapeFactory::instance()->createLine(Vec2d(0, 0), Vec2d(6.5, 8.5));
        std::cout << l3->to_string() << std::endl;

        auto &&l4 = ShapeFactory::instance()->createLine(
            Vec2d(0, 0), cada::Math::deg2rad(45), sqrt(2));
        std::cout << l4->to_string() << std::endl;

        auto &&l5 = ShapeFactory::instance()->createLine(
            Vec2d(0, 0), cada::Math::deg2rad(180 + 45), sqrt(2));
        std::cout << l5->to_string() << std::endl;
    }
    // arc
    {
        auto &&a1 = ShapeFactory::instance()->createArc();
        print_arc_information(a1.release());

        auto &&a2 = ShapeFactory::instance()->createArc(
            Vec2d(0, 0), 5, 0, cada::Math::deg2rad(90));
        print_arc_information(a2.release());

        auto &&a3 = ShapeFactory::instance()->createArcFrom3Point(
            Vec2d(5.00000000, 0.00000000), Vec2d(3.53553391, 3.53553391),
            Vec2d(0.00000000, 5.00000000));
        print_arc_information(a3.release());
    }

    return 0;
}