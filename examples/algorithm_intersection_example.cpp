#include <cada_shape.h>
#include <cada_ns.h>
#include <cada_math.h>
#include <iostream>
#include <iomanip>

using namespace cada::shape;

int main(int argc, char** argv)
{
    // ll
    // {
    //     auto&& l1 = ShapeFactory::instance()->createLine(Vec2d(0,0), Vec2d(10,10));
    //     auto&& l2 = ShapeFactory::instance()->createLine(Vec2d(0, 10), Vec2d(10, 0));
    //     auto &&l3 = ShapeFactory::instance()->createLine(Vec2d(3, 1), Vec2d(6, 0));
    //     auto&& ips12 = l1->getIntersectionPoints(l2.get());
    //     for(auto&& ip : ips12)
    //     {
    //         std::cout << ip.to_string() << std::endl;
    //     }
    //     auto&& ips13 = l1->getIntersectionPoints(l3.get());
    //     for(auto&& ip : ips13)
    //     {
    //         std::cout << ip.to_string() << std::endl;
    //     }
    //     auto &&ips13_not_limited = l1->getIntersectionPoints(l3.get(), false);
    //     for (auto &&ip : ips13_not_limited) 
    //     {
    //         std::cout << ip.to_string() << std::endl;
    //     }
    // }

    // // la
    // {
    //     auto&& l1 = ShapeFactory::instance()->createLine(Vec2d(0,0), Vec2d(10,10));
    //     auto&& a1 = ShapeFactory::instance()->createArc(Vec2d(0,0), 5, 0, cada::Math::deg2rad(90));
    //     auto&& ipsl = l1->getIntersectionPoints(a1.get());
    //     for(auto&& ip : ipsl)
    //     {
    //         std::cout << ip.to_string() << std::endl;
    //     }
    //     auto&& ipsnl = l1->getIntersectionPoints(a1.get(), false);
    //     for(auto&& ip : ipsnl)
    //     {
    //         std::cout << ip.to_string() << std::endl;
    //     }    
    // }

    // aa
    // {
    //     auto&& a1 = ShapeFactory::instance()->createArc(Vec2d(0,0), 5, 0, cada::Math::deg2rad(90));
    //     auto&& a2 = ShapeFactory::instance()->createArc(Vec2d(10,0), 10, cada::Math::deg2rad(45), cada::Math::deg2rad(180));
    //     auto&& ipsl = a1->getIntersectionPoints(a2.get());
    //     for(auto&& ip : ipsl)
    //     {
    //         std::cout << ip.to_string() << std::endl;
    //     }
    //     auto&& ipsnl = a1->getIntersectionPoints(a2.get(), false);
    //     for(auto&& ip : ipsnl)
    //     {
    //         std::cout << ip.to_string() << std::endl;
    //     }
    // }  
  
    // ac
    {
        auto&& a1 = ShapeFactory::instance()->createArc(Vec2d(0,0), 5, 0, cada::Math::deg2rad(90));
        auto&& c2 = ShapeFactory::instance()->createCircle(Vec2d(10,0), 10);
        auto&& ipsl = a1->getIntersectionPoints(c2.get());
        for(auto&& ip : ipsl)
        {
            std::cout << ip.to_string() << std::endl;
        }
        auto&& ipsnl = a1->getIntersectionPoints(c2.get(), false);
        for(auto&& ip : ipsnl)
        {
            std::cout << ip.to_string() << std::endl;
        } 
    }

    return 0;
}