#pragma once

#include <SFML/Graphics.hpp>
#include <bitset>
#include <array>
#include <memory>

#include "Vec2.hpp"

class CTransform
{
public:
    
    Vec2 p = { 0.0, 0.0 };
    Vec2 v = { 0.0, 0.0 };
    Vec2 a = { 0.0, 0.0 };
    bool moved = false;

    CTransform() {}
    CTransform(const Vec2 & pin) : p(pin) {}
};

class CCircleBody
{
public:
    double r = 10;
    double m = 0;
    bool collided = true;

    CCircleBody() {}
    CCircleBody(double radius)
        : r(radius), m(radius * 10) { }
};

class CCircleShape
{
public:
    sf::CircleShape shape;
    CCircleShape() {}
    CCircleShape(double radius)
        : shape((float)radius, 32)
    {
        shape.setOrigin((float)radius, (float)radius);
    }
};

class GridSensor;
class PuckSensor;
class ObstacleSensor;
class RobotSensor;
class CSensorArray
{
public:
    std::vector<std::shared_ptr<GridSensor>>     gridSensors;
    std::vector<std::shared_ptr<GridSensor>>     oppGridSensors;
    std::vector<std::shared_ptr<PuckSensor>>     puckSensors;
    std::vector<std::shared_ptr<ObstacleSensor>> obstacleSensors;
    std::vector<std::shared_ptr<RobotSensor>>    robotSensors;
    CSensorArray() {}
};

class CLineBody
{
public:
    Vec2 s;
    Vec2 e;
    double r = 1.0;

    CLineBody() {}

    CLineBody(Vec2 start, Vec2 end, double radius)
        : s(start), e(end), r(radius) { }
};


class CRobotType
{
public:
    size_t type = 0;
    CRobotType(size_t t = 0)
        : type(t) {}
};

class CSteer
{
public:
    double angle = 0;
    double speed = 0;
    CSteer() {}
};

class CColor
{
public:
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;
    uint8_t a = 255;
    CColor() {}
    CColor(int rr, int gg, int bb, int aa)
        : r((uint8_t)rr), g((uint8_t)gg), b((uint8_t)bb), a((uint8_t)aa) {}
};

// Used to draw a vector from a robot (or potentially other entity)
class CVectorIndicator
{
public:
    double angle, length;

    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;
    uint8_t a = 255;
    CVectorIndicator() {}
    CVectorIndicator(double a, double l, int rr, int gg, int bb, int aa)
        : angle(a), length(l), r((uint8_t)rr), g((uint8_t)gg), b((uint8_t)bb), a((uint8_t)aa) { }
};

// Simulates a triangular plow attached to the front of a robot.
class CPlowBody
{
public:
    double width, length;
    sf::ConvexShape shape;

    CPlowBody() {}
    CPlowBody(double w, double l)
        : width(w)
        , length(l)
        , shape()
    {
        shape.setPointCount(3);
        shape.setPoint(0, sf::Vector2f(0, -w/2.0f));
        shape.setPoint(1, sf::Vector2f(l, 0));
        shape.setPoint(2, sf::Vector2f(0, w/2.0f));
    }
};

class EntityController;
class CController
{
public:
    std::shared_ptr<EntityController> controller;
    CController() {}
    CController(decltype(controller) c)
        : controller(c) {}
};
