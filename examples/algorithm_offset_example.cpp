#include <cada_shape.h>
#include <cada_ns.h>
#include <cada_math.h>
#include <iostream>
#include <iomanip>

using namespace cada::shape;

int main(int argc, char **argv)
{
    // // l
    // {
    //     auto &&l = ShapeFactory::instance()->createLine(Vec2d(0, 0), Vec2d(10, 10));
    //     auto &&offset213 = l->getOffsetShapes(2, 1, cada::NS::Side::BothSides);
    //     for (auto &&offset : offset213) {
    //         std::cout << offset->to_string() << std::endl;
    //     }

    //     auto &&offset221 = l->getOffsetShapes(2, 2, cada::NS::Side::LeftHand);
    //     for (auto &&offset : offset221) {
    //         std::cout << offset->to_string() << std::endl;
    //     }
    // }

    // arc
    {
        auto&& a = ShapeFactory::instance()->createArc(Vec2d(0,0), 5, 0, cada::Math::deg2rad(90));
        auto &&offset213 = a->getOffsetShapes(2, 1, cada::NS::Side::BothSides);
        for (auto &&offset : offset213) {
            std::cout << offset->to_string() << std::endl;
        }

        auto &&offset221 = a->getOffsetShapes(2, 2, cada::NS::Side::LeftHand);
        for (auto &&offset : offset221) {
            std::cout << offset->to_string() << std::endl;
        } 
    }

    return 0;
}