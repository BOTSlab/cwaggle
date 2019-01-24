#pragma once

#include <SFML/Graphics.hpp>
#include <bitset>
#include <array>

#include "Vec2.hpp"

class CTransform2
{
public:
    
    Vec2 p = { 0.0, 0.0 };
    Vec2 v = { 0.0, 0.0 };
    Vec2 a = { 0.0, 0.0 };
    bool moved = false;

    CTransform2() {}
    CTransform2(const Vec2 & pin) : p(pin) {}
};

class CBody
{
public:
    Vec2 p = { 0.0, 0.0 };
    Vec2 v = { 0.0, 0.0 };
    Vec2 a = { 0.0, 0.0 };
    double r = 10;
    double m = 0;
    bool moved = false;
    bool collided = false;

    CBody() {}
    CBody(const Vec2 & pos, double radius)
        : p(pos), r(radius), m(radius * 10) {}
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

class CShape
{
public:
    sf::CircleShape shape;

    CShape() {}
    CShape(double radius, const sf::Color & color)
        : shape((float)radius, 32)
    {
        shape.setFillColor(color);
        shape.setOrigin((float)radius, (float)radius);
    }
};


class CLifeSpan
{
public:
    size_t created = 0;
    size_t lifespan = 0;
    CLifeSpan() {}
    CLifeSpan(int currentFrame, int l) : created(currentFrame), lifespan(l) {}
};

class CInput
{
public:
    bool up         = false;
    bool down       = false;
    bool left       = false;
    bool right      = false;
    bool shoot      = false;
    bool canShoot   = true;

    CInput() {}
};

class CBoundingBox
{
public:
    Vec2 size;
    Vec2 halfSize;
    CBoundingBox() {}
    CBoundingBox(const Vec2 & s)
        : size(s), halfSize(s.x / 2, s.y / 2) {}
};

class CGravity
{
public:
    float gravity;
    CGravity() : gravity(0) {}
    CGravity(float g) : gravity(g) {}
};