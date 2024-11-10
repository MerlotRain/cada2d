#include <cada_shape.h>
#include <cada_ns.h>
#include <cada_math.h>
#include <iostream>
#include <iomanip>

using namespace cada::shape;

void point_base_method()
{
    auto &&p = ShapeFactory::instance()->createPoint();
    p->setPosition(Vec2d(1, 2));
    bool v = p->isValid();

    // end points
    auto &&end_points = p->getEndPoints();
    std::cout << "end points:" << std::endl;
    for (auto &&end_point : end_points) {
        std::cout << end_point.to_string() << std::endl;
    }
    // middle points
    auto &&middle_points = p->getMiddlePoints();
    std::cout << "middle points:" << std::endl;
    for (auto &&middle_point : middle_points) {
        std::cout << middle_point.to_string() << std::endl;
    }
    // center points
    auto &&center_points = p->getCenterPoints();
    std::cout << "center points:" << std::endl;
    for (auto &&center_point : center_points) {
        std::cout << center_point.to_string() << std::endl;
    }

    // point information
    std::cout << p->to_string() << std::endl;
}

void line_base_method_1()
{
    auto &&l = ShapeFactory::instance()->createLine(Vec2d(0, 0), Vec2d(1, 1));
    std::cout << std::boolalpha << l->isValid() << std::endl;
    std::cout << l->to_string() << std::endl;

    // end points
    auto &&end_points = l->getEndPoints();
    std::cout << "end points:" << std::endl;
    for (auto &&end_point : end_points) {
        std::cout << end_point.to_string() << std::endl;
    }
    // middle points
    auto &&middle_points = l->getMiddlePoints();
    std::cout << "middle points:" << std::endl;
    for (auto &&middle_point : middle_points) {
        std::cout << middle_point.to_string() << std::endl;
    }
    // center points
    auto &&center_points = l->getCenterPoints();
    std::cout << "center points:" << std::endl;
    for (auto &&center_point : center_points) {
        std::cout << center_point.to_string() << std::endl;
    }

    auto &&start_piont = l->getStartPoint();
    auto &&end_point = l->getEndPoint();
    auto &&middle_point = l->getMiddlePoint();
    std::cout << "start point:" << start_piont.to_string() << std::endl;
    std::cout << "end point:" << end_point.to_string() << std::endl;
    std::cout << "middle point:" << middle_point.to_string() << std::endl;

    auto &&length = l->getLength();
    std::cout << "length:" << length << std::endl;
    auto &&angle = l->getAngle();
    std::cout << "angle:" << angle << std::endl;
}

void line_base_method_2()
{
    auto &&l1 = ShapeFactory::instance()->createLine(Vec2d(0, 0), Vec2d(1, 1));
    auto &&l2 = ShapeFactory::instance()->createLine(Vec2d(5, 5), Vec2d(7, 7));
    auto &&l3 = ShapeFactory::instance()->createLine(Vec2d(2, 2), Vec2d(3, 5));

    auto parallel12 = l1->isParallel(l2.get());
    auto parallel13 = l1->isParallel(l3.get());
    std::cout << "parallel12:" << std::boolalpha << parallel12 << std::endl;
    std::cout << "parallel13:" << std::boolalpha << parallel13 << std::endl;

    std::cout << l1->to_string() << std::endl;
    std::cout << l2->to_string() << std::endl;
    std::cout << l3->to_string() << std::endl;
}

void line_base_method_3()
{
    auto &&l1 = ShapeFactory::instance()->createLine(Vec2d(1, 0), Vec2d(1, 8));
    auto &&l2 = ShapeFactory::instance()->createLine(Vec2d(2, 7), Vec2d(7, 7));

    std::cout << "l1 direction: is horizontal: " << std::boolalpha
              << l1->isHorizontal() << "; is vertical: " << l1->isVertical()
              << std::endl;
    std::cout << "l2 direction: is horizontal: " << std::boolalpha
              << l2->isHorizontal() << "; is vertical: " << l2->isVertical()
              << std::endl;
}

void arc_base_method1()
{
    auto &&arc = ShapeFactory::instance()->createArc(
        Vec2d(0, 0), 5, cada::Math::deg2rad(0), cada::Math::deg2rad(90));
    std::cout << std::boolalpha << arc->isValid() << std::endl;
    std::cout << arc->to_string() << std::endl;

    // end points
    auto &&end_points = arc->getEndPoints();
    std::cout << "end points:" << std::endl;
    for (auto &&end_point : end_points) {
        std::cout << end_point.to_string() << std::endl;
    }
    // middle points
    auto &&middle_points = arc->getMiddlePoints();
    std::cout << "middle points:" << std::endl;
    for (auto &&middle_point : middle_points) {
        std::cout << middle_point.to_string() << std::endl;
    }
    // center points
    auto &&center_points = arc->getCenterPoints();
    std::cout << "center points:" << std::endl;
    for (auto &&center_point : center_points) {
        std::cout << center_point.to_string() << std::endl;
    }

    // sweep angle
    auto &&sweep_angle = arc->getSweep();
    std::cout << "sweep angle:" << cada::Math::rad2deg(sweep_angle)
              << std::endl;
    // bulge
    auto &&bulge = arc->getBulge();
    std::cout << "bulge:" << bulge << std::endl;

    // arc reference point
    auto &&arc_reference_point = arc->getArcRefPoints();
    std::cout << "arc reference point:" << std::endl;
    for (auto &&arc_ref_point : arc_reference_point) {
        std::cout << arc_ref_point.to_string() << std::endl;
    }
}

void arc_base_method2()
{
    auto &&arc = ShapeFactory::instance()->createArc(Vec2d(0, 0), 5, 0, M_PI_2);
    std::cout << "begin point: " << arc->getStartPoint().to_string()
              << std::endl;
    std::cout << "end point: " << arc->getEndPoint().to_string() << std::endl;
    std::cout << "center point: " << arc->getCenter().to_string() << std::endl;
    std::cout << "radius: " << arc->getRadius() << std::endl;

    arc->moveStartPoint(Vec2d(6, 0), false);

    std::cout << "begin point: " << arc->getStartPoint().to_string()
              << std::endl;
    std::cout << "end point: " << arc->getEndPoint().to_string() << std::endl;
    std::cout << "center point: " << arc->getCenter().to_string() << std::endl;
    std::cout << "radius: " << arc->getRadius() << std::endl;
}

int main(int argc, char **argv)
{
    // point
    // std::cout << "point base method:" << std::endl;
    // point_base_method();
    // std::cout << std::endl;
    // std::cout << std::endl;
    // // line
    // std::cout << "line base method:" << std::endl;
    // line_base_method_1();
    // std::cout << std::endl;
    // line_base_method_2();
    // std::cout << std::endl;
    // line_base_method_3();
    // std::cout << std::endl;
    // std::cout << std::endl;
    // arc
    std::cout << "arc base method:" << std::endl;
    // arc_base_method1();
    arc_base_method2();

    return 0;
}