#pragma once

#include "Simulator.hpp"
#include "EntityControllers.hpp"
#include "World.hpp"
#include "ValueGrid.hpp"
#include "Sensors.hpp"

#include "configs.hpp"

#include <sstream>

using namespace std;

namespace sort_world
{
    void addRobot(std::shared_ptr<World> world, ExperimentConfig config)
    {
        // Note: The position assigned to the robot will later be overwritten.

        Entity robot = world->addEntity("robot");

        Vec2 rPos(0, 0);
        robot.addComponent<CTransform>(rPos);
        if (!config.fakeRobots) {
            robot.addComponent<CCircleBody>(config.robotRadius, true);
            robot.addComponent<CCircleShape>(config.robotRadius);
            robot.addComponent<CColor>(255, 100, 100, 255);
        }
        //robot.addComponent<CRobotType>(robotType);

        int robotRadius = (int) config.robotRadius;

        // Wedge shaped plow at the front of the robot.
        if (config.plowLength > 0)
            robot.addComponent<CPlowBody>(config.plowLength, config.robotRadius);

        auto & sensors = robot.addComponent<CSensorArray>();

        // Grid sensors at three points on the distance transform (grid 0 and 1)
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 0, 0, 0));
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 0, 0, config.plowLength));
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 0, 90, config.plowLength));
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 1, 0, 0));
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 1, 0, config.plowLength));
        sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 1, 90, config.plowLength));

        // Measure the x- and y- gradients at the robot's centre (grids 1 and 2).
        //sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 1, 0, 0));
        //sensors.gridSensors.push_back(std::make_shared<GridSensor>(robot, 2, 0, 0));

        //
        // TERRITORY
        // 

        // The coordinates of the territory will get overwritten when the world's contents are randomized.
        robot.addComponent<CTerritory>(0, 0, 200);

        //
        // PUCK SENSOR FOR TERRITORIAL SORTING
        // 

        // Both the puck sensor and robot sensor are instances of FancySensor.  Both will use the same initial
        // circle (C1) which represents the camera's given field-of-view.  Note that this is not in any way
        // a good model of the field-of-view of the actual robots.


        std::vector<SensingCircle> leftCircles;
    
        // C1: The basic sensing circle, centred in front of the robot
        leftCircles.push_back(SensingCircle(0, config.c1Distance, config.c1Radius, true));

        // C2: A large circle to the left, effectively allowing only the left half of C1 to be visible. 
        leftCircles.push_back(SensingCircle(-M_PI/2.0, config.c2Distance, config.c2Radius, true));

        // A circle that prevents seeing any pucks outside the territory's radius.
        leftCircles.push_back(SensingCircle(0, 0, 0, true, 0, true));

        auto fancyLeftRedPuckSensor = std::make_shared<FancySensor>(robot, "red_puck", "left", leftCircles);//, true, false, 0, 0, true, 0, false, false);
        auto fancyLeftGreenPuckSensor = std::make_shared<FancySensor>(robot, "green_puck", "left", leftCircles);//, true, false, 0, 0, true, 0, false, false);
        sensors.fancySensors.push_back(fancyLeftRedPuckSensor);
        sensors.fancySensors.push_back(fancyLeftGreenPuckSensor);

        // We'll also make right sensors, which is necessary to detect that there are pucks on both the
        // left and right and that the robot needs to move to the periphery.

        std::vector<SensingCircle> rightCircles(leftCircles);
        rightCircles[1].m_angle = M_PI/2.0;
    
        auto fancyRightRedPuckSensor = std::make_shared<FancySensor>(robot, "red_puck", "right", rightCircles);
        auto fancyRightGreenPuckSensor = std::make_shared<FancySensor>(robot, "green_puck", "right", rightCircles);
        sensors.fancySensors.push_back(fancyRightRedPuckSensor);
        sensors.fancySensors.push_back(fancyRightGreenPuckSensor);

        //
        // ROBOT SENSOR
        // 

        std::vector<SensingCircle> robotCircles;

        // A circle that prevents seeing any pucks outside the territory's radius.
        robotCircles.push_back(SensingCircle(0, 0, 0, true, 0, true, 4*robotRadius));

        auto fancyRobotSensor = std::make_shared<FancySensor>(robot, "robot", "both", robotCircles);
        sensors.fancySensors.push_back(fancyRobotSensor);
    }

    void addRobots(std::shared_ptr<World> world, size_t width, size_t height, ExperimentConfig config)
    {
        int numRobots = (int) config.numRobots;
        for (size_t r = 0; r < numRobots; r++)
        {
            addRobot(world, config);
        }
    }

    void addPuck(std::string name, size_t red, size_t green, size_t blue, std::shared_ptr<World> world, size_t puckRadius)
    {
        // Note: The position assigned to the robot will later be overwritten.

        Entity puck = world->addEntity(name);
        Vec2 pPos(0, 0);
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
            addPuck(name, red, green, blue, world, puckRadius);
        }
    }

    std::shared_ptr<World> GetWorld(ExperimentConfig config)
    {
        cout << "Reminder: GetWorld creates assets but their positions are overwritten." << endl;
        ValueGrid valueGrid0(config.grid0Filename);
        ValueGrid valueGrid1(config.grid1Filename);
        //ValueGrid valueGrid2(config.grid2Filename);

        size_t width = valueGrid0.width();
        size_t height = valueGrid0.height();

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

        // Corner pieces.
        /*
        double length = 25;
        t = length / sqrt(2.0);
        Entity topLeftCorner = world->addEntity("line");
        Entity botLeftCorner = world->addEntity("line");
        Entity topRightCorner = world->addEntity("line");
        Entity botRightCorner = world->addEntity("line");
        topLeftCorner.addComponent<CLineBody>(Vec2(-length, 2*length), Vec2(2*length, -length), t);
        botLeftCorner.addComponent<CLineBody>(Vec2(-length, height-2*length), Vec2(2*length, height+length), t);
        topRightCorner.addComponent<CLineBody>(Vec2(width+length, 2*length), Vec2(width-2*length, -length), t);
        botRightCorner.addComponent<CLineBody>(Vec2(width+length, height-2*length), Vec2(width-2*length, height+length), t);
        */

        addRobots(world, width, height, config);

        addPucks("red_puck", 200, 44, 44, world, width, height, config);
        addPucks("green_puck", 44, 200, 44, world, width, height, config);

        world->addGrid(valueGrid0);
        world->addGrid(valueGrid1);
        //world->addGrid(valueGrid2);

        world->update();
        return world;
    }    
};
