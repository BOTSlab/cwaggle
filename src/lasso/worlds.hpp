#pragma once

#include "EntityControllers.hpp"
#include "Simulator.hpp"
#include "ValueGrid.hpp"
#include "World.hpp"
#include "WorldUtils.hpp"

#include "configs.hpp"

#include <sstream>

using namespace std;

namespace lasso_world {

Entity addRobot(std::shared_ptr<World> world, ExperimentConfig config)
{
    Entity robot = world->addEntity("robot");

    // This position will later be overwritten.
    Vec2 rPos(0, 0);
    robot.addComponent<CTransform>(rPos);
    if (!config.fakeRobots) {
        robot.addComponent<CCircleBody>(config.robotRadius, true);
        robot.addComponent<CCircleShape>(config.robotRadius);
        robot.addComponent<CColor>(255, 100, 100, 255);
    }
    //robot.addComponent<CRobotType>(robotType);

    int robotRadius = (int)config.robotRadius;

    auto& sensors = robot.addComponent<CSensorArray>();

    // Grid sensors at three points with the most forward point at the tip of the plow.
    double plowAngleRad = config.plowAngleDeg * M_PI / 180.0;

    // Wedge shaped plow at the front of the robot.
    if (config.plowLength > 0)
        robot.addComponent<CPlowBody>(config.plowLength, config.robotRadius, config.plowAngleDeg);

    // Position of plow tip (f for forward sensor).
    double fx = config.plowLength * cos(plowAngleRad) - config.puckRadius;
    double fy = config.plowLength * sin(plowAngleRad) + config.puckRadius;

    double forwardAngleDeg = (180 / M_PI) * atan2(fy, fx);
    double forwardDist = hypot(fy, fx);
    double centreAngleDeg = (180 / M_PI) * atan2(fy, fx - config.plowLength);
    double centreDist = hypot(fy, fx - config.plowLength);
    double rightAngleDeg = (180 / M_PI) * atan2(fy + config.robotRadius, fx - config.plowLength);
    double rightDist = hypot(fy + config.robotRadius, fx - config.plowLength);

    sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridForward0", 0, forwardAngleDeg, forwardDist));
    sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridCentre0", 0, centreAngleDeg, centreDist));
    sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridRight0", 0, rightAngleDeg, rightDist));
    sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridRight0", 0, rightAngleDeg, rightDist));

    return robot;
}

void addRobots(shared_ptr<World> world, uniform_int_distribution<int> Xrng, uniform_int_distribution<int> Yrng, default_random_engine rng,
    ExperimentConfig config)
{
    int numRobots = (int)config.numRobots;
    for (size_t r = 0; r < numRobots; r++) {
        Entity robot = addRobot(world, config);
        auto& transform = robot.getComponent<CTransform>();
        transform.p.x = config.robotRadius + Xrng(rng);
        transform.p.y = config.robotRadius + Yrng(rng);
    }
}

Entity addPuck(string name, size_t red, size_t green, size_t blue, shared_ptr<World> world, size_t puckRadius)
{
    Entity puck = world->addEntity(name);

    // This position will later be overwritten.
    Vec2 pPos(0, 0);
    puck.addComponent<CTransform>(pPos);
    puck.addComponent<CCircleBody>(puckRadius);
    puck.addComponent<CCircleShape>(puckRadius);
    puck.addComponent<CColor>(red, green, blue, 255);

    return puck;
}

void addPucks(string name, size_t red, size_t green, size_t blue, shared_ptr<World> world, uniform_int_distribution<int> Xrng, uniform_int_distribution<int> Yrng, default_random_engine rng,
    ExperimentConfig config)
{
    int puckRadius = (int)config.puckRadius;
    for (size_t r = 0; r < config.numPucks; r++) {
        Entity puck = addPuck(name, red, green, blue, world, puckRadius);

        auto& transform = puck.getComponent<CTransform>();
        transform.p.x = config.puckRadius + Xrng(rng);
        transform.p.y = config.puckRadius + Yrng(rng);

        //std::cerr << puck.id() << ", x, y: " << transform.p.x << ", " << transform.p.y << std::endl;
    }
}

shared_ptr<World> GetWorld(default_random_engine rng, ExperimentConfig config)
{
    ValueGrid valueGrid0(config.grid0Filename, 1.0);
    ValueGrid valueGrid1(config.grid1Filename, 0.0);
    ValueGrid valueGrid2(config.grid2Filename, 1.0);

    size_t width = valueGrid0.width();
    size_t height = valueGrid0.height();

    auto world = std::make_shared<World>(width, height);

    // Create the left and right arcs to match our stadium-shaped air hockey table
    WorldUtils::AddLineBodyArc(world, 36, width/3, height/2, height/2, -3*M_PI/2, -M_PI/2, 100);
    WorldUtils::AddLineBodyArc(world, 36, 2*width/3, height/2, height/2, -M_PI/2, M_PI/2, 100);

    // Create top and bottom line bodies.  These are not really needed for
    // the simulation aspect, but moreso for visualization.
    Entity topWall = world->addEntity("line");
    Entity botWall = world->addEntity("line");
    topWall.addComponent<CLineBody>(Vec2(width/3, 0), Vec2(2*width/3, 0), 1);
    botWall.addComponent<CLineBody>(Vec2(width/3, height+1), Vec2(2*width/3, height+1), 1);

    // An island obstacle
    //Entity island = world->addEntity("line");
    //island.addComponent<CLineBody>(Vec2(3*width/4 - 20, height/4), Vec2(3*width/4 + 20, height/4), 16);

    // Obstacle walls
    if (config.wallArrangement >= 1) {
        Entity leftObsWall = world->addEntity("line");
        leftObsWall.addComponent<CLineBody>(Vec2(width / 3, 0), Vec2(width / 3, height / 2), 16);

        Entity rightObsWall = world->addEntity("line");
        rightObsWall.addComponent<CLineBody>(Vec2(2 * width / 3, height), Vec2(2 * width / 3, 3 * height / 4), 16);
    }
    if (config.wallArrangement == 2) {
        Entity leftObsWall2 = world->addEntity("line");
        leftObsWall2.addComponent<CLineBody>(Vec2(width / 3, height / 2), Vec2(width / 2, height / 2), 16);

        Entity rightObsWall2 = world->addEntity("line");
        rightObsWall2.addComponent<CLineBody>(Vec2(2 * width / 3, 3 * height / 4), Vec2(width / 2, 3 * height / 4), 16);
    }

    // Prepare to generate random x and y positions for robots.
    int randXDomain = world->width() - 2 * (int)config.robotRadius;
    int randYDomain = world->height() - 2 * (int)config.robotRadius;
    uniform_int_distribution<int> robotXrng(0, randXDomain);
    uniform_int_distribution<int> robotYrng(0, randYDomain);

    addRobots(world, robotXrng, robotYrng, rng, config);

    // Prepare to generate random x and y positions for pucks.
    // Random number generators for x- and y- position.
    randXDomain = world->width() - 2 * (int)config.puckRadius;
    randYDomain = world->height() - 2 * (int)config.puckRadius;
    uniform_int_distribution<int> puckXrng(0, randXDomain);
    uniform_int_distribution<int> puckYrng(0, randYDomain);

    addPucks("red_puck", 200, 44, 44, world, puckXrng, puckYrng, rng, config);
    //addPucks("green_puck", 44, 200, 44, world, puckXrng, puckYrng, rng, config);

    world->addGrid(valueGrid0);
    world->addGrid(valueGrid1);
    world->addGrid(valueGrid2);

    // Three grids (red, green, and blue) for visualization.
    world->addGrid(ValueGrid{valueGrid0.width(), valueGrid0.height(), 0, 0});
    world->addGrid(ValueGrid{valueGrid0.width(), valueGrid0.height(), 0, 0});
    world->addGrid(ValueGrid{valueGrid0.width(), valueGrid0.height(), 0, 0});

    world->update();
    return world;
}

};
