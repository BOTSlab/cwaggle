#pragma once

#include "EntityControllers.hpp"
#include "Simulator.hpp"
#include "ValueGrid.hpp"
#include "World.hpp"
#include "WorldUtils.hpp"

#include "configs.hpp"

#include <sstream>

using namespace std;

namespace uorbit_world {

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

    return robot;
}

void addRobots(shared_ptr<World> world, uniform_int_distribution<int> Xrng, uniform_int_distribution<int> Yrng, default_random_engine rng,
    ExperimentConfig config)
{
    int nRobots = (int) config.nRobots;
    for (size_t r = 0; r < nRobots; r++) {
        Entity robot = addRobot(world, config);
        auto& transform = robot.getComponent<CTransform>();
        transform.p.x = config.robotRadius + Xrng(rng);
        transform.p.y = config.robotRadius + Yrng(rng);
    }
}

// A probe is a robot without a body or a controller.
Entity addProbe(std::shared_ptr<World> world, ExperimentConfig config)
{
    Entity probe = world->addEntity("probe");

    // This position will later be overwritten.
    Vec2 rPos(world->width()/2, world->height()/2);
    probe.addComponent<CTransform>(rPos);
    probe.addComponent<CCircleBody>(5, true);
    probe.addComponent<CCircleShape>(5);
    probe.addComponent<CColor>(100, 100, 255, 100);

    return probe;
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

void addPucks(string name, size_t red, size_t green, size_t blue, size_t nPucks, shared_ptr<World> world, uniform_int_distribution<int> Xrng, uniform_int_distribution<int> Yrng, default_random_engine rng,
    ExperimentConfig config)
{
    int puckRadius = (int)config.puckRadius;
    for (size_t r = 0; r < nPucks; r++) {
        Entity puck = addPuck(name, red, green, blue, world, puckRadius);

        auto& transform = puck.getComponent<CTransform>();
        transform.p.x = config.puckRadius + Xrng(rng);
        transform.p.y = config.puckRadius + Yrng(rng);

        //std::cerr << puck.id() << ", x, y: " << transform.p.x << ", " << transform.p.y << std::endl;
    }
}

shared_ptr<World> GetWorld(default_random_engine rng, ExperimentConfig config)
{
    ValueGrid valueGrid0(config.grid0Filename, 0.5);
    ValueGrid valueGrid1(config.grid1Filename, 0.5);
    ValueGrid valueGrid2(config.grid2Filename, 0.5);
    ValueGrid valueGrid3(config.grid3Filename, 0.5);

    size_t width = valueGrid0.width();
    size_t height = valueGrid0.height();

    auto world = std::make_shared<World>(width, height);

    bool stadium = false;
    if (stadium) {
        // Create the left and right arcs to match our stadium-shaped air hockey table
        WorldUtils::AddLineBodyArc(world, 36, width/3, height/2, height/2, -3*M_PI/2, -M_PI/2, 100);
        WorldUtils::AddLineBodyArc(world, 36, 2*width/3, height/2, height/2, -M_PI/2, M_PI/2, 100);

        // Create top and bottom line bodies.  These are not really needed for
        // the simulation aspect, but moreso for visualization.
        Entity topWall = world->addEntity("line");
        Entity botWall = world->addEntity("line");
        topWall.addComponent<CLineBody>(Vec2(width/3, 0), Vec2(2*width/3, 0), 1);
        botWall.addComponent<CLineBody>(Vec2(width/3, height+1), Vec2(2*width/3, height+1), 1);
    } else {
        Entity topWall = world->addEntity("line");
        Entity botWall = world->addEntity("line");
        Entity leftWall = world->addEntity("line");
        Entity rightWall = world->addEntity("line");
        topWall.addComponent<CLineBody>(Vec2(0, 0), Vec2(width, 0), 1);
        botWall.addComponent<CLineBody>(Vec2(0, height), Vec2(width, height), 1);
        leftWall.addComponent<CLineBody>(Vec2(0, 0), Vec2(0, height), 1);
        rightWall.addComponent<CLineBody>(Vec2(width, 0), Vec2(width, height), 1);
    }

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

    addProbe(world, config);

    Entity redGoal = world->addEntity("red_goal");
    redGoal.addComponent<CTransform>(Vec2{config.redGoalX, config.redGoalY});
    redGoal.addComponent<CCircleShape>(config.redGoalRadius);
    redGoal.addComponent<CColor>(255, 0, 0, 50);

    Entity greenGoal = world->addEntity("green_goal");
    greenGoal.addComponent<CTransform>(Vec2{config.greenGoalX, config.greenGoalY});
    greenGoal.addComponent<CCircleShape>(config.greenGoalRadius);
    greenGoal.addComponent<CColor>(0, 255, 0, 50);

    // Prepare to generate random x and y positions for pucks.
    // Random number generators for x- and y- position.
    randXDomain = world->width() - 2 * (int)config.puckRadius;
    randYDomain = world->height() - 2 * (int)config.puckRadius;
    uniform_int_distribution<int> puckXrng(0, randXDomain);
    uniform_int_distribution<int> puckYrng(0, randYDomain);

    addPucks("red_puck", 200, 44, 44, config.nRedPucks, world, puckXrng, puckYrng, rng, config);
    addPucks("green_puck", 44, 200, 44, config.nGreenPucks, world, puckXrng, puckYrng, rng, config);

    world->addGrid(valueGrid0);
    world->addGrid(valueGrid1);
    world->addGrid(valueGrid2);
    world->addGrid(valueGrid3);

    // Three more grids (red, green, and blue) for visualization.
    world->addGrid(ValueGrid{valueGrid0.width(), valueGrid0.height(), 0, 0});
    world->addGrid(ValueGrid{valueGrid0.width(), valueGrid0.height(), 0, 0});
    world->addGrid(ValueGrid{valueGrid0.width(), valueGrid0.height(), 0, 0});

    world->update();
    return world;
}

};
