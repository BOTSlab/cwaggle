#pragma once

#include "Simulator.hpp"
#include "EntityControllers.hpp"
#include "World.hpp"
#include "ValueGrid.hpp"
#include "Sensors.hpp"

#include <sstream>

namespace orbital_av_world
{
    // Parameters of the obstacle created in GetWallWorld
//    int leftX = 399 + 25;
//    int rightX = 499 - 25;
//    int topY = 0;
//    int bottomY = 399 - 25;
//    int obsPad = 20;

//    int leftX = 399;
//    int rightX = 419;
//    int topY = 0;
//    int bottomY = 399;

    int obsPad = 20;
    int obsX = 399;
    int obsTopY = 0;
    int obsBottomY = 399;
    int obsRadius = 5;

    bool positionOkay(size_t x, size_t y) {
//        return !(x + obsPad > leftX && x - obsPad < rightX && y + obsPad > topY && y - obsPad < bottomY);
//        int p = obsRadius + obsPad;
//        return !(x > obsX - p && x < obsX + p && y > obsTopY - p && y < obsBottomY + p);
return true;
    }

    void addRobots(bool teamRed, std::shared_ptr<World> world, size_t width, size_t height, size_t numRobots, size_t robotRadius, size_t senseRadius)
    {
        size_t red, green, blue;
        size_t gridIndex, otherGridIndex;
        std::string puckType, otherPuckType;
        if (teamRed) {
            red = 255;
            green = 100;
            blue = 100;
            gridIndex = 0;
            otherGridIndex = 1;
            puckType = "red_puck";
            otherPuckType = "green_puck";
        } else {
            red = 100;
            green = 255;
            blue = 100;
            gridIndex = 1;
            otherGridIndex = 0;
            puckType = "green_puck";
            otherPuckType = "red_puck";
        }
 
        for (size_t r = 0; r < numRobots; r++)
        {
            Entity robot = world->addEntity("robot");

            bool positionFree = false;
            int x, y;
            while (!positionFree) {
                x = rand() % width;
                y = rand() % height;

                positionFree = positionOkay(x, y);
            }

            Vec2 rPos(x, y);
            robot.addComponent<CTransform>(rPos);
            robot.addComponent<CCircleBody>(robotRadius);
            robot.addComponent<CCircleShape>(robotRadius);
            robot.addComponent<CColor>(red, green, blue, 255);
            robot.addComponent<CRobotType>(0);

            auto & sensors = robot.addComponent<CSensorArray>();
            double gridSensorRadius = senseRadius;
            double puckSensorRadius = 2 * senseRadius;
            double obsSensorRadius = 2;
            double puckSensorAngle = 55;
            sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, gridIndex, 45, robotRadius + gridSensorRadius));
            sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, gridIndex, 0, robotRadius + gridSensorRadius));
            sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, gridIndex, -45, robotRadius + gridSensorRadius));
            sensors.oppGridSensors.push_back(std::make_shared<GridSensor>(robot, otherGridIndex, 45, robotRadius + gridSensorRadius));
            sensors.oppGridSensors.push_back(std::make_shared<GridSensor>(robot, otherGridIndex, 0, robotRadius + gridSensorRadius));
            sensors.oppGridSensors.push_back(std::make_shared<GridSensor>(robot, otherGridIndex, -45, robotRadius + gridSensorRadius));
            sensors.puckSensors.push_back(std::make_shared<PuckSensor>(robot, puckType, -puckSensorAngle, 
                                                                       robotRadius + puckSensorRadius, puckSensorRadius));
            sensors.puckSensors.push_back(std::make_shared<PuckSensor>(robot, otherPuckType, puckSensorAngle, 
                                                                       robotRadius + puckSensorRadius, puckSensorRadius));
//            sensors.obstacleSensors.push_back(std::make_shared<ObstacleSensor>(robot, 45, robotRadius + obsSensorRadius, obsSensorRadius));
            sensors.obstacleSensors.push_back(std::make_shared<ObstacleSensor>(robot, -45, robotRadius + obsSensorRadius, obsSensorRadius));
        }
    }

    void addPucks(std::string name, size_t red, size_t green, size_t blue, std::shared_ptr<World> world, size_t width, size_t height, size_t numPucks, size_t puckSize)
    {
//std::cout << "PUCKS" << std::endl;
        for (size_t r = 0; r < numPucks; r++)
        {
            bool positionFree = false;
            int x, y;
            while (!positionFree) {
                x = 4*puckSize + rand() % (int)(width - 8 * puckSize);
                y = 4*puckSize + rand() % (int)(height - 8 * puckSize);

                positionFree = positionOkay(x, y);
            }

            Vec2 pPos(x, y);
//std::cout << x << "," << y << std::endl;
//assert(!isnan(x));            
//assert(!isnan(y));            

            Entity puck = world->addEntity(name);
            puck.addComponent<CTransform>(pPos);
            puck.addComponent<CCircleBody>(puckSize);
            puck.addComponent<CCircleShape>(puckSize);
            puck.addComponent<CColor>(red, green, blue, 255);
        }
    }

    std::shared_ptr<World> GetSimpleWorld(size_t numRobots, double robotRadius, double senseRadius, size_t numPucks, double puckSize)
    {
        ValueGrid valueGrid("images/potential_field_simple.png");
        size_t width = valueGrid.width();
        size_t height = valueGrid.height();

        auto world = std::make_shared<World>(width, height);

        addRobots(true, world, width, height, numRobots, robotRadius, senseRadius);

        addPucks("red_puck", 200, 44, 44, world, width, height, numPucks, puckSize);

        world->addGrid(valueGrid);

        world->update();
        return world;
    }    

    std::shared_ptr<World> GetWallWorld(size_t numRobots, double robotRadius, double senseRadius, size_t numPucks, double puckSize)
    {
        ValueGrid valueGrid("images/potential_field_wall.png");
        size_t width = valueGrid.width();
        size_t height = valueGrid.height();

        auto world = std::make_shared<World>(width, height);

        addRobots(true, world, width, height, numRobots, robotRadius, senseRadius);

        // add lines representing an obstacle
        Entity line = world->addEntity("line");
        line.addComponent<CLineBody>(Vec2(obsX, obsTopY), Vec2(obsX, obsBottomY), obsRadius);

        /* RECTANGULAR OBSTACLE BUTTED UP AGAINST TOP OF SCREEN        
        Entity line1 = world->addEntity("line");
        line1.addComponent<CLineBody>(Vec2(leftX, topY), Vec2(leftX, bottomY), 1);

        Entity line2 = world->addEntity("line");
        line2.addComponent<CLineBody>(Vec2(rightX, topY), Vec2(rightX, bottomY), 1);
        
        Entity line3 = world->addEntity("line");
        line3.addComponent<CLineBody>(Vec2(leftX, bottomY), Vec2(rightX, bottomY), 1);
        */
 
        /** TRIANGULAR OBSTACLE
        Entity line1 = world->addEntity("line");
        line1.addComponent<CLineBody>(Vec2(leftX, bottomY), Vec2((rightX+leftX)/2, topY), 1);

        Entity line2 = world->addEntity("line");
        line2.addComponent<CLineBody>(Vec2(rightX, bottomY), Vec2((rightX+leftX)/2, topY), 1);
        */

        addPucks("red_puck", 200, 44, 44, world, width, height, numPucks, puckSize);

        world->addGrid(valueGrid);

        world->update();
        return world;
    }    

    std::shared_ptr<World> GetSymmetricWorld(size_t numRobots, double robotRadius, double senseRadius, size_t numPucks, double puckSize)
    {
        ValueGrid valueGridRed("images/potential_field_red.png");
        ValueGrid valueGridGreen("images/potential_field_green.png");
        size_t width = valueGridRed.width();
        size_t height = valueGridRed.height();

        auto world = std::make_shared<World>(width, height);

        addRobots(true, world, width, height, numRobots/2, robotRadius, senseRadius);
        addRobots(false, world, width, height, numRobots/2, robotRadius, senseRadius);

        addPucks("red_puck", 200, 44, 44, world, width, height, numPucks, puckSize);
        addPucks("green_puck", 44, 200, 44, world, width, height, numPucks, puckSize);

        world->addGrid(valueGridRed);
        world->addGrid(valueGridGreen);

        world->update();
        return world;
    }

    std::shared_ptr<World> GetBarsWorld(size_t numRobots, double robotRadius, double senseRadius, size_t numPucks, double puckSize)
    {
        ValueGrid valueGridRed("images/potential_field_redBar.png");
        ValueGrid valueGridGreen("images/potential_field_greenBar.png");
        size_t width = valueGridRed.width();
        size_t height = valueGridRed.height();

        auto world = std::make_shared<World>(width, height);

        addRobots(true, world, width, height, numRobots/2, robotRadius, senseRadius);
        addRobots(false, world, width, height, numRobots/2, robotRadius, senseRadius);

        addPucks("red_puck", 200, 44, 44, world, width, height, numPucks, puckSize);
        addPucks("green_puck", 44, 200, 44, world, width, height, numPucks, puckSize);

        world->addGrid(valueGridRed);
        world->addGrid(valueGridGreen);

        world->update();
        return world;
    }
};