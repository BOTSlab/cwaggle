#pragma once

#include "CWaggle.h"
#include "EntityControllers.hpp"
#include "OrbitJudge.hpp"
#include "MySensors.hpp"
#include "SensorTools.hpp"
#include "configs.hpp"
#include <math.h>
#include <random>

using namespace std;

class EntityController_UOrbit : public EntityController {
    std::shared_ptr<World> m_world;
    Entity m_robot;
    ControllerConfig m_config;
    CTerritory m_territory;
    OrbitJudge m_orbitJudge;

    CControllerVis& m_visComponent;
    CVectorIndicator m_indicator;

public:
    EntityController_UOrbit(Entity robot, std::shared_ptr<World> world, ControllerConfig config, ExperimentConfig experimentConfig)
        : m_world(world)
        , m_robot(robot)
        , m_config(config)
        , m_territory(m_robot.getComponent<CTransform>().p, 30)
        , m_orbitJudge(world, experimentConfig)
        , m_visComponent(m_robot.addComponent<CControllerVis>())
        , m_indicator(3.14, 20, 255, 0, 0, 255)
    {
        m_robot.addComponent<CTerritory>(m_territory);
    }

    void resetIndicator() {
        m_indicator.angle = 0;
        m_indicator.length = 0;
        m_indicator.r = 0;
        m_indicator.g = 0;
        m_indicator.b = 0;
        m_indicator.a = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);
    }

    virtual EntityAction getAction()
    {
        resetIndicator();

        updateCentre();

        Vec2 robotPos = m_robot.getComponent<CTransform>().p;
        double robotAngle = m_robot.getComponent<CSteer>().angle;
        double distToCentre = robotPos.dist(m_territory.centre);

        double epsilon = (fmin(distToCentre, 2 * m_territory.radius) - m_territory.radius) / m_territory.radius;
        //cerr << "epsilon: " << epsilon << endl;

        double dx = m_territory.centre.x - robotPos.x;
        double dy = m_territory.centre.y - robotPos.y;
        double angleToCentre = atan2(dy, dx) - robotAngle;

        double K = 0.9;
        double idealAngle = M_PI/2 - K * epsilon;

        // Using https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        // to compute the signed delta angle x - y
        double deltaAngle = idealAngle - angleToCentre;
        double angleError = -atan2(sin(deltaAngle), cos(deltaAngle));

        // Forward and angular speeds in the range [-1, 1].
        double v = cos(angleError);
        double w = sin(angleError);

        return EntityAction(v * m_config.maxForwardSpeed, w * m_config.maxAngularSpeed);
    }

    void updateCentre() {
        using XYR = tuple<double, double, double>;

        // tuples will contain all of the circles to test, including the current one.
        auto currentCircle = make_tuple(m_territory.centre.x, m_territory.centre.y, m_territory.radius);
        vector<XYR> tuples{ currentCircle };

        int skip = 4;
        /*
        for (int i=0; i < m_world->width(); i += skip)
            for (int j=0; j < m_world->height(); j += skip)
                tuples.push_back(make_tuple(i, j, m_territory.radius));
        */
        int width = m_world->width();
        int height = m_world->height();
        int centreSearchRadius = 20;
        int radiusSearchRadius = 5;
        int centreSearchSkip = 1;
        int radiusSearchSkip = 1;
        for (int i=-centreSearchRadius; i <= centreSearchRadius; i += centreSearchSkip)
            for (int j=-centreSearchRadius; j <= centreSearchRadius; j += centreSearchSkip)
                for (int k=-radiusSearchRadius; k <= radiusSearchRadius; k += radiusSearchSkip) {
                    double x = m_territory.centre.x + i;
                    double y = m_territory.centre.y + j;
                    double r = m_territory.radius + k;
                    if (x >= 0 && x < width && y >= 0 && y < height && r > 1)
                        tuples.push_back(make_tuple(x, y, r));
                }

        // A score is assigned for each tuple
        map<XYR, double> scores;
        for (XYR t : tuples)
            scores[t] = m_orbitJudge.getScore(m_robot, t, m_territory.centre);

        // Find the <circle, score> with the highest score that exceeds the score
        // of the current circle.
        auto best = make_pair(currentCircle, scores[currentCircle]);

        for (auto it = scores.begin(); it != scores.end(); ++it)
            if (it->second > best.second)
                best = *it;
        m_territory.centre.x = std::get<0>(best.first);
        m_territory.centre.y = std::get<1>(best.first);
        m_territory.radius = std::get<2>(best.first);

        //cerr << "x, y, radius, score: " << m_territory.centre.x << ", " << m_territory.centre.y << ", " << m_territory.radius << ", " << best.second << endl;
        m_robot.addComponent<CTerritory>(m_territory);
    }
};