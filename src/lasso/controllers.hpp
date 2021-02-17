#pragma once

#include "CWaggle.h"
#include "EntityControllers.hpp"
#include "MySensors.hpp"
#include "SensorTools.hpp"
#include "configs.hpp"
#include <math.h>
#include <random>

using namespace std;

class EntityController_Lasso : public EntityController {
    std::shared_ptr<World> m_world;
    Entity m_robot;
    ControllerConfig m_config;

    GlobalSensor m_globalSensor;

    // Maximum value seen for (forward - centre)
    double m_maxAbsFminusC;

    SensorReading m_reading;

    CControllerVis& m_visComponent;
    CVectorIndicator m_indicator;

public:
    double m_targetThreshold;

    EntityController_Lasso(Entity robot, std::shared_ptr<World> world, ControllerConfig config)
        : m_world(world)
        , m_robot(robot)
        , m_config(config)
        , m_globalSensor(robot)
        , m_maxAbsFminusC(0)
        , m_visComponent(m_robot.addComponent<CControllerVis>())
        , m_indicator(3.14, 20, 255, 0, 0, 255)
    {
        m_targetThreshold = 0.15;
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
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        // Values from the three grid sensors.
        double forward = m_reading.gridForward0;
        double centre = m_reading.gridCentre0;
        double right = m_reading.gridRight0;

        bool puckValid = false;
        double puckValue = m_globalSensor.getExtremeGridValue(m_world, m_robot, "red_puck", true, 0, 1, m_config.fieldOfView, m_config.maxSensingDistance, puckValid);

        //bool robotAhead = m_globalSensor.anotherRobotAhead(m_world, m_robot, M_PI/2.0, m_config.fieldOfView, m_config.robotReactDistance);
vector<pair<double, double>> otherRobotIntervals = m_globalSensor.getOtherRobotIntervals(m_world, m_robot, M_PI/2.0, m_config.fieldOfView, m_config.robotReactDistance);
double highestOtherRobotValue = 0;
for (pair<double, double> p : otherRobotIntervals)
    if (p.second > highestOtherRobotValue)
        highestOtherRobotValue = p.second;
bool reactToRobot = otherRobotIntervals.size() > 0 && right < highestOtherRobotValue;


        if (reactToRobot)
            m_targetThreshold += 0.01;
        else {
//            m_targetThreshold -= 0.01;
            if (puckValid)
                m_targetThreshold = puckValue;
        }


        double epsilon = 0;
        if (m_targetThreshold != 0)
            epsilon = (fmin(forward, 2 * m_targetThreshold) - m_targetThreshold) / m_targetThreshold;

        // Forward and angular speed, to be further specified below.
        double v = 1;
        double w = 0;

        if (forward >= 1)
            w = 1.0;
        else {
            if (right > centre) {
                v = 0;
                if (forward > centre)
                    w = 1;
                else
                    w = -1;
            } else {
                w = epsilon;
            }
        }

//        if (robotAhead)
//            w = -0.5;

        //
        // Debug / visualization
        //

        if (reactToRobot)
            m_robot.addComponent<CColor>(255, 0, 0, 255);
        else
            m_robot.addComponent<CColor>(0, 255, 0, 255);
        
        if (m_robot.getComponent<CControllerVis>().selected) {
            Vec2 robotPos = m_robot.getComponent<CTransform>().p;

            m_robot.addComponent<CTerritory>(robotPos, m_config.robotReactDistance);

            auto & visGrid = m_world->getGrid(5);
            visGrid.addContour(m_targetThreshold, m_world->getGrid(0), 1.0);

            if (w > 0)
                m_indicator.angle = M_PI/2.0;
            else if (w < 0)
                m_indicator.angle = -M_PI/2.0;
            m_indicator.length = 50;
            m_robot.addComponent<CVectorIndicator>(m_indicator);
            std::stringstream ss;
            ss << m_reading.toString()
            << "epsilon: " << epsilon << endl;
            //<< "v, w: " << v << ", " << w << endl;
            m_visComponent.msg = ss.str();
        } else {
            // This doesn't work:
            // m_robot.removeComponent<CTerritory>();
            // So we use this hack of adding an off-screen territory instead
            m_robot.addComponent<CTerritory>(Vec2{-1000, -1000}, 0);
        }

        return EntityAction(v * m_config.maxForwardSpeed, w * m_config.maxAngularSpeed);
    }
};