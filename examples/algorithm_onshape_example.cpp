#include <cada_shape.h>
#include <cada_ns.h>
#include <cada_math.h>
#include <iostream>
#include <iomanip>

using namespace cada::shape;

int main(int argc, char **argv)
{

    Vec2d t1(0, 0);
    Vec2d t2(5, 5);
    Vec2d t3(3.53553391, 3.53553391);
    Vec2d t4(10, 10);
    Vec2d t5(8, 9);

    // point
    {
        auto &&p1 = ShapeFactory::instance()->createPoint();
        std::cout << t1.to_string() << " is on point1: " << std::boolalpha
                  << p1->isOnShape(t1) << std::endl;
        std::cout << t2.to_string() << " is on point1: " << std::boolalpha
                  << p1->isOnShape(t2) << std::endl;
        std::cout << t3.to_string() << " is on point1: " << std::boolalpha
                  << p1->isOnShape(t3) << std::endl;

        auto &&p2 =
            ShapeFactory::instance()->createPoint(3.53553391, 3.53553391);    
        std::cout << t1.to_string() << " is on point2: " << std::boolalpha
                  << p2->isOnShape(t1) << std::endl;
        std::cout << t2.to_string() << " is on point2: " << std::boolalpha
                  << p2->isOnShape(t2) << std::endl;
        std::cout << t3.to_string() << " is on point2: " << std::boolalpha
                  << p2->isOnShape(t3) << std::endl;
    }

    // line
    {
        auto &&l1 = ShapeFactory::instance()->createLine();
        std::cout << t1.to_string() << " is on line1: " << std::boolalpha
                  << l1->isOnShape(t1) << std::endl;
        std::cout << t2.to_string() << " is on line1: " << std::boolalpha
                  << l1->isOnShape(t2) << std::endl;
        std::cout << t3.to_string() << " is on line1: " << std::boolalpha
                  << l1->isOnShape(t3) << std::endl;
    
    
        auto &&l2 = ShapeFactory::instance()->createLine(t1, t2);
        std::cout << t1.to_string() << " is on line2: " << std::boolalpha
                  << l2->isOnShape(t1) << std::endl;
        std::cout << t2.to_string() << " is on line2: " << std::boolalpha
                  << l2->isOnShape(t2) << std::endl;
        std::cout << t3.to_string() << " is on line2: " << std::boolalpha
                  << l2->isOnShape(t3) << std::endl;
        std::cout << t4.to_string() << " is on line2: " << std::boolalpha
                  << l2->isOnShape(t4) << std::endl;
        std::cout << t4.to_string() << " is on line2: " << std::boolalpha
                  << l2->isOnShape(t4, false) << std::endl;
    }

    // arc
    {
        auto &&a1 = ShapeFactory::instance()->createArc();
        std::cout << t1.to_string() << " is on arc1: " << std::boolalpha
                  << a1->isOnShape(t1) << std::endl;
        std::cout << t2.to_string() << " is on arc1: " << std::boolalpha
                  << a1->isOnShape(t2) << std::endl;
        std::cout << t3.to_string() << " is on arc1: " << std::boolalpha
                  << a1->isOnShape(t3) << std::endl;

        auto &&a2 = ShapeFactory::instance()->createArc(
            Vec2d(0, 0), 5, 0, cada::Math::deg2rad(120));
        t3 = Vec2d(3.5355339059327378, 3.5355339059327373);
        Vec2d t6(0, 5);
        Vec2d t7(-5, 0);
        std::cout << t1.to_string() << " is on arc2: " << std::boolalpha
                  << a2->isOnShape(t1) << std::endl;
        std::cout << t2.to_string() << " is on arc2: " << std::boolalpha
                  << a2->isOnShape(t2) << std::endl;
        std::cout << t3.to_string() << " is on arc2: " << std::boolalpha
                  << a2->isOnShape(t3) << std::endl;
        std::cout << t6.to_string() << " is on arc2: " << std::boolalpha
                  << a2->isOnShape(t6) << std::endl;
        std::cout << t7.to_string() << " is on arc2: " << std::boolalpha
                  << a2->isOnShape(t7) << std::endl;
        std::cout << t7.to_string() << " is on arc2: " << std::boolalpha
                  << a2->isOnShape(t7, false) << std::endl;
    }

    return 0;
}