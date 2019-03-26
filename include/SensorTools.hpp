#pragma once

#include <sstream>

#include "Entity.hpp"
#include "World.hpp"
#include "Sensors.hpp"

struct SensorReading
{
    // left, mid, right grid sensor readings
    double leftNest = 0;
    double midNest = 0;
    double rightNest = 0;

    // left, mid, right sensor readings for the opposite grid (i.e. green if sorting red pucks)
    double leftOppNest = 0;
    double midOppNest = 0;
    double rightOppNest = 0;

    // puck sensor readings
    double leftPucks = 0;
    double rightPucks = 0;

    // obstacle sensor readings
    double leftObstacle = 0;
    double rightObstacle = 0;

    std::string toString()
    {
        std::stringstream ss;
        ss << "lNest: " << leftNest << "\n";
        ss << "mNest: " << midNest << "\n";
        ss << "rNest: " << rightNest << "\n";
        ss << "lOppNest: " << leftOppNest << "\n";
        ss << "mOppNest: " << midOppNest << "\n";
        ss << "rOppNest: " << rightOppNest << "\n";
        ss << "lPuck: " << leftPucks << "\n";
        ss << "rPuck: " << rightPucks << "\n";
        ss << "lObst: " << leftObstacle << "\n";
        ss << "rObst: " << rightObstacle;
        return ss.str();
    }
};

namespace SensorTools
{
    inline void ReadSensorArray(Entity e, std::shared_ptr<World> world, SensorReading & reading)
    {
        reading = {};

        auto & sensors = e.getComponent<CSensorArray>();
        for (auto & sensor : sensors.gridSensors)
        {
            if (sensor->angle() < 0) { reading.leftNest = sensor->getReading(world); }
            if (sensor->angle() > 0) { reading.rightNest = sensor->getReading(world); }
            if (sensor->angle() == 0) { reading.midNest = sensor->getReading(world); }
        }
        for (auto & sensor : sensors.oppGridSensors)
        {
            if (sensor->angle() < 0) { reading.leftOppNest = sensor->getReading(world); }
            if (sensor->angle() > 0) { reading.rightOppNest = sensor->getReading(world); }
            if (sensor->angle() == 0) { reading.midOppNest = sensor->getReading(world); }
        }
        for (auto & sensor : sensors.obstacleSensors)
        {
            if (sensor->angle() <= 0) { reading.leftObstacle += sensor->getReading(world); }
            if (sensor->angle() > 0) { reading.rightObstacle += sensor->getReading(world); }
        }
        for (auto & sensor : sensors.puckSensors)
        {
            if (sensor->angle() <= 0) { reading.leftPucks += sensor->getReading(world); }
            if (sensor->angle() > 0) { reading.rightPucks += sensor->getReading(world); }
        }
    }

    
}