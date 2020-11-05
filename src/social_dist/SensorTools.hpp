#pragma once

#include <sstream>

#include "Entity.hpp"
#include "World.hpp"
#include "Sensors.hpp"

struct SensorReading
{
    // grid sensor readings
    double leftNest = 0;
    double midNest = 0;
    double rightNest = 0;

    double gridForward0 = 0;
    double gridCentre0 = 0;
    double gridRight0 = 0;
    double gridForward1 = 0;
    double gridCentre1 = 0;
    double gridRight1 = 0;

    // puck sensor readings
    double leftRedPucks = 0;
    double leftGreenPucks = 0;
    double rightRedPucks = 0;
    double rightGreenPucks = 0;

    // obstacle sensor readings
//    double leftObstacle = 0;
//    double rightObstacle = 0;

    // robot sensor readings
//    double leftRobots = 0;
//    double rightRobots = 0;
    double robots = 0;

//    std::vector<bool> image;

    std::string toString()
    {
        std::stringstream ss;
        ss << "leftNest: " << leftNest << "\n";
        ss << "midNest: " << midNest << "\n";
        ss << "rightNest: " << rightNest << "\n";

        ss << "gridForward0: " << gridForward0 << "\n";
        ss << "gridCentre0: " << gridCentre0 << "\n";
        ss << "gridRight0: " << gridRight0 << "\n";
        ss << "gridForward1: " << gridForward1 << "\n";
        ss << "gridCentre1: " << gridCentre1 << "\n";
        ss << "gridRight1: " << gridRight1 << "\n";

        ss << "leftRedPucks: " << leftRedPucks << "\n";
        ss << "leftGreenPucks: " << leftGreenPucks << "\n";
        ss << "rightRedPucks: " << rightRedPucks << "\n";
        ss << "rightGreenPucks: " << rightGreenPucks << "\n";
//        ss << "lObst: " << leftObstacle << "\n";
//        ss << "rObst: " << rightObstacle << "\n";
//        ss << "leftRobots: " << leftRobots << "\n";
//        ss << "rightRobots: " << rightRobots;

        /*
        for (std::vector<bool>::const_iterator i = image.begin(); i != image.end(); ++i)
            ss << *i << ' ';
        ss << "\n";
        */

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

            if (sensor->m_gridIndex == 0) {
                if (sensor->distance() == 0) 
                    reading.gridCentre0 = sensor->getReading(world);
                else if (sensor->angle() == 0) 
                    reading.gridForward0 = sensor->getReading(world);
                else if (sensor->angle() > 0) 
                    reading.gridRight0 = sensor->getReading(world);
            } else if (sensor->m_gridIndex == 1) {
                if (sensor->distance() == 0) 
                    reading.gridCentre1 = sensor->getReading(world);
                else if (sensor->angle() == 0) 
                    reading.gridForward1 = sensor->getReading(world);
                else if (sensor->angle() > 0) 
                    reading.gridRight1 = sensor->getReading(world);
            }

        }
        /*
        for (auto & sensor : sensors.obstacleSensors)
        {
            if (sensor->angle() <= 0) { reading.leftObstacle += sensor->getReading(world); }
            if (sensor->angle() > 0) { reading.rightObstacle += sensor->getReading(world); }
        }
        for (auto & sensor : sensors.robotSensors)
        {
            if (sensor->angle() <= 0) { reading.leftRobot += sensor->getReading(world); }
            if (sensor->angle() > 0) { reading.rightRobot += sensor->getReading(world); }
        }
        */
        /*
        for (auto & sensor : sensors.puckSensors)
        {
            if (sensor->angle() <= 0) { reading.leftRedPucks += sensor->getReading(world); }
            if (sensor->angle() > 0) { reading.rightRedPucks += sensor->getReading(world); }
        }
        */

        for (auto & sensor : sensors.fancySensors) {
            if (sensor->m_typeName == "red_puck" && sensor->m_name == "left") 
                reading.leftRedPucks += sensor->getReading(world);
            if (sensor->m_typeName == "red_puck" && sensor->m_name == "right") 
                reading.rightRedPucks += sensor->getReading(world);
            if (sensor->m_typeName == "green_puck" && sensor->m_name == "left") 
                reading.leftGreenPucks += sensor->getReading(world);
            if (sensor->m_typeName == "green_puck" && sensor->m_name == "right") 
                reading.rightGreenPucks += sensor->getReading(world);
            if (sensor->m_typeName == "robot") 
                reading.robots += sensor->getReading(world);
            
            //if (sensor->m_typeName == "robot" && sensor->m_name == "left") 
            //    reading.leftRobots += sensor->getReading(world);
            //if (sensor->m_typeName == "robot" && sensor->m_name == "right") 
            //    reading.rightRobots += sensor->getReading(world);
        }
        /*
        for (auto & sensor : sensors.cameraSensors) {
            reading.image = sensor->getReading(world);
        }
        */
    }
}