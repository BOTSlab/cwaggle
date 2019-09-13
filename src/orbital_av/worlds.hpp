#pragma once

#include "Simulator.hpp"
#include "EntityControllers.hpp"
#include "World.hpp"
#include "ValueGrid.hpp"
#include "Sensors.hpp"

#include "configs.hpp"

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
//      return !(x + obsPad > leftX && x - obsPad < rightX && y + obsPad > topY && y - obsPad < bottomY);
//        int p = obsRadius + obsPad;
//        return !(x > obsX - p && x < obsX + p && y > obsTopY - p && y < obsBottomY + p);
return true;
    }

    void addRobot(std::string team, std::shared_ptr<World> world, size_t x, size_t y, ExperimentConfig config)
    {
        size_t red, green, blue;
        size_t gridIndex, otherGridIndex;
        std::string puckType, otherPuckType;
        if (team == "teamRed") {
            red = 255;
            green = 100;
            blue = 100;
            gridIndex = 0;
            otherGridIndex = 1;
            puckType = "red_puck";
            otherPuckType = "green_puck";
        } else if (team == "teamGreen") {
            red = 100;
            green = 255;
            blue = 100;
            gridIndex = 1;
            otherGridIndex = 0;
            puckType = "green_puck";
            otherPuckType = "red_puck";
        } else if (team == "justRed") {
            red = 100;
            green = 100;
            blue = 255;
            gridIndex = 0;
            puckType = "red_puck";
        }
 
        Entity robot = world->addEntity("robot");

        Vec2 rPos(x, y);
        robot.addComponent<CTransform>(rPos);
        robot.addComponent<CCircleBody>(config.robotRadius, true);
        robot.addComponent<CCircleShape>(config.robotRadius);
        robot.addComponent<CColor>(red, green, blue, 255);
        //robot.addComponent<CRobotType>(robotType);

        int robotRadius = (int) config.robotRadius;

        // Wedge shaped plow at the front of the robot.
        if (config.plowLength > 0) {
            float plowWidth = 2*robotRadius;
            robot.addComponent<CPlowBody>(plowWidth, config.plowLength, 0);
        }

        auto & sensors = robot.addComponent<CSensorArray>();
        double gridSensorRadius = 5;
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, gridIndex, 45, robotRadius + gridSensorRadius));
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, gridIndex, 0, robotRadius + gridSensorRadius));
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, gridIndex, -45, robotRadius + gridSensorRadius));
        if (team != "justRed") {
            sensors.oppGridSensors.push_back(std::make_shared<GridSensor>(robot, otherGridIndex, 45, robotRadius + gridSensorRadius));
            sensors.oppGridSensors.push_back(std::make_shared<GridSensor>(robot, otherGridIndex, 0, robotRadius + gridSensorRadius));
            sensors.oppGridSensors.push_back(std::make_shared<GridSensor>(robot, otherGridIndex, -45, robotRadius + gridSensorRadius));
        }

        // Both the puck sensor and robot sensor are instances of FancySensor.  Both will use the same initial
        // circle (C1) which represents the camera's given field-of-view.  Note that this is not in any way
        // a good model of the field-of-view of the actual robots.

        //
        // PUCK SENSOR
        // 

        // The puck sensor has an additional circle (C2) representing an intentionally contracted field-of-view
        // so that the robot does not react to pucks outside of its action space.
        std::vector<SensingCircle> circles;
        circles.push_back(SensingCircle(0, config.c1Distance, config.c1Radius, true));
        circles.push_back(SensingCircle(0, config.c2Distance, config.c2Radius, true));

        auto fancyLeftPuckSensor = std::make_shared<FancySensor>(robot, "red_puck", "left", circles, true, false);
        sensors.fancySensors.push_back(fancyLeftPuckSensor);

        //
        // ROBOT SENSOR
        // 

        auto cameraSensor = std::make_shared<PseudoCameraSensor>(robot, "robot", 2*robotRadius, 3*robotRadius, 0.75 * M_PI, 6);
        sensors.cameraSensors.push_back(cameraSensor);

        /*
        circles.clear();
        circles.push_back(SensingCircle(0, config.c1Distance, config.c1Radius, true));
        circles.push_back(SensingCircle(0, 0, config.c3Radius, true));
        auto fancyRightRobotSensor = std::make_shared<FancySensor>(robot, "robot", "right", circles, false, true);
        sensors.fancySensors.push_back(fancyRightRobotSensor);

        auto fancyLeftRobotSensor = std::make_shared<FancySensor>(robot, "robot", "left", circles, true, false);
        sensors.fancySensors.push_back(fancyLeftRobotSensor);
        */
    }

    void addRobots(std::string team, std::shared_ptr<World> world, size_t width, size_t height, ExperimentConfig config)
    {
        int numRobots = (int) config.numRobots;
        for (size_t r = 0; r < numRobots; r++)
        {
            bool positionFree = false;
            int x, y;
            while (!positionFree) {
                x = rand() % width;
                y = rand() % height;                
                positionFree = positionOkay(x, y);
            }

            addRobot(team, world, x, y, config);
        }
    }

    void addPuck(std::string name, size_t red, size_t green, size_t blue, std::shared_ptr<World> world, size_t x, size_t y, size_t puckRadius)
    {
        Entity puck = world->addEntity(name);
        Vec2 pPos(x, y);
        puck.addComponent<CTransform>(pPos);
        puck.addComponent<CCircleBody>(puckRadius);
        puck.addComponent<CCircleShape>(puckRadius);
        puck.addComponent<CColor>(red, green, blue, 255);
    }

    void addPucks(std::string name, size_t red, size_t green, size_t blue, std::shared_ptr<World> world, size_t width, size_t height, ExperimentConfig config)
    {
        int puckRadius = (int) config.puckRadius;
        for (size_t r = 0; r < config.numPucks; r++)
        {
            bool positionFree = false;
            int x, y;
            while (!positionFree) {
                x = puckRadius + rand() % (int)(width - 2 * puckRadius);
                y = puckRadius + rand() % (int)(height - 2 * puckRadius);

                positionFree = positionOkay(x, y);
            }

            addPuck(name, red, green, blue, world, x, y, puckRadius);
        }
    }

    std::shared_ptr<World> GetLGTVWorld(ExperimentConfig config)
    {
        ValueGrid valueGrid(config.gridFilename);
        size_t width = valueGrid.width();
        size_t height = valueGrid.height();

        auto world = std::make_shared<World>(width, height);

        // Create the boundary walls.
        Entity topWall = world->addEntity("line");
        Entity bottomWall = world->addEntity("line");
        Entity leftWall = world->addEntity("line");
        Entity rightWall = world->addEntity("line");
        // Thickness (divided by 2) of the walls which are positioned outside the visible area
        double t = width / 5.0;
        topWall.addComponent<CLineBody>(Vec2(-t, -t + 1), Vec2(width-1 + t, -t + 1), t);
        bottomWall.addComponent<CLineBody>(Vec2(-t, height-1 + t), Vec2(width-1 + t, height-1 + t), t);
        leftWall.addComponent<CLineBody>(Vec2(-t, -t), Vec2(-t, height-1 + t), t);
        rightWall.addComponent<CLineBody>(Vec2(width-1 + t, -t), Vec2(width-1 + t, height-1 + t), t);

        addRobots("justRed", world, width, height, config);
        //addRobot("justRed", world, width/2, height/2, robotRadius, senseRadius, hookSensor);

        addPucks("red_puck", 200, 44, 44, world, width, height, config);
        //addPuck("red_puck", 200, 44, 44, world, width-puckSize, height/2, puckSize);

        world->addGrid(valueGrid);

        world->update();
        return world;
    }    

    /*
    std::shared_ptr<World> GetWallWorld(size_t numRobots, double robotRadius, double senseRadius, size_t numPucks, double puckSize)
    {
        ValueGrid valueGrid("images/potential_field_wall.png");
        size_t width = valueGrid.width();
        size_t height = valueGrid.height();

        auto world = std::make_shared<World>(width, height);

        addRobots("teamRed", world, width, height, numRobots, robotRadius, senseRadius);

        // add lines representing an obstacle
        Entity line = world->addEntity("line");
        line.addComponent<CLineBody>(Vec2(obsX, obsTopY), Vec2(obsX, obsBottomY), obsRadius);

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

        addRobots("teamRed", world, width, height, numRobots/2, robotRadius, senseRadius);
        //addRobots("teamGreen", world, width, height, numRobots/2, robotRadius, senseRadius);

        addPucks("red_puck", 200, 44, 44, world, width, height, numPucks, puckSize);
        //addPucks("green_puck", 44, 200, 44, world, width, height, numPucks, puckSize);

        world->addGrid(valueGridRed);
        world->addGrid(valueGridGreen);

        world->update();
        return world;
    }
    */
};
