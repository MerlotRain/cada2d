#include <cada_shape.h>
#include <iostream>

#include <gtest/gtest.h>

using namespace cada::shape;

TEST(arcTest, SimpleConstructor)
{
    auto arc = ShapeFactory::instance()->createArc(Vec2d(0, 0), 5, 0, 90);
    EXPECT_EQ(arc->getRadius(), 5);
    EXPECT_EQ(arc->getStartAngle(), 0);
    EXPECT_EQ(arc->getEndAngle(), 90);
}