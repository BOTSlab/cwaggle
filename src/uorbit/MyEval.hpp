#pragma once

#include "World.hpp"
#include "Entity.hpp"
#include "Components.hpp"
#include "configs.hpp"

namespace UOrbitEval
{
    int pucksInGoal(std::shared_ptr<World> world, std::string puckType, Vec2 goalCentre, double goalRadius)
    {
        int n = 0;
        for (auto e : world->getEntities(puckType))
            if (e.getComponent<CTransform>().p.dist(goalCentre) < goalRadius)
                n++;
        return n;
    }

    double evaluate(std::shared_ptr<World> world, ExperimentConfig config) {
        double nRedInGoal = pucksInGoal(world, "red_puck", Vec2{config.redGoalX, config.redGoalY}, config.redGoalRadius);
        double nGreenInGoal = pucksInGoal(world, "green_puck", Vec2{config.greenGoalX, config.greenGoalY}, config.greenGoalRadius);

        if (config.nRedPucks > 0 && config.nGreenPucks > 0)
            return 0.5 * (nRedInGoal / config.nRedPucks + nGreenInGoal / config.nGreenPucks);
        if (config.nRedPucks > 0)
            return nRedInGoal / config.nRedPucks;
        if (config.nGreenPucks > 0)
            return nGreenInGoal / config.nGreenPucks;
        else
            // Must not be any pucks!
            return 0;
    }
}