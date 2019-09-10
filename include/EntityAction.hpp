#pragma once

#include "Entity.hpp"
#include "Components.hpp"

#include <cassert>

class EntityAction
{
public:
    double  m_angularSpeed = 0;
    double  m_speed        = 0;
        
    EntityAction() {}
    EntityAction(double speed, double angle)
        : m_angularSpeed(angle), m_speed(speed) {}

    inline const auto speed()        const { return m_speed; }
    inline const auto angularSpeed() const { return m_angularSpeed; }

    virtual void doAction(Entity e, double timeStep)
    {
        if (!e.hasComponent<CSteer>())
        {
            e.addComponent<CSteer>();
        }
        auto & steer = e.getComponent<CSteer>();

        if (steer.frozen) {
            steer.speed = 0;
            return;
        }

        if (steer.slowedCount > 0)
            steer.angle += 0.1 * m_angularSpeed * timeStep;
        else
            steer.angle += m_angularSpeed * timeStep;

        steer.speed  = m_speed;
    }
};