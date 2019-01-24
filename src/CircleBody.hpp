#pragma once

#include "Vec2.hpp"

#include <stdint.h>
#include <array>
#include <vector>

class CircleBody
{
public:
    Vec2    p       = { 0, 0 }; // position
    Vec2    v       = { 0, 0 }; // velocity
    Vec2    a       = { 0, 0 }; // acceleration
    double  m       = 0;        // mass
    double  r       = 0;        // radius
    size_t  id      = 0;        // id
    bool    moved   = false;    // moved last update
    bool    collided = true;   // collided last update

    CircleBody() {}

    CircleBody(const Vec2 & position, double radius, int identifier)
        : p(position)
        , m(radius * 10)
        , r(radius)
        , id(identifier)
    {

    }
};