#pragma once

#include "CWaggle.h"
#include "EntityControllers.hpp"
#include "MySensors.hpp"
#include "SensorTools.hpp"
#include "configs.hpp"
#include <math.h>
#include <random>

using namespace std;

class EntityController_MCO : public EntityController {
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

    EntityController_MCO(Entity robot, std::shared_ptr<World> world, ControllerConfig config)
        : m_world(world)
        , m_robot(robot)
        , m_config(config)
        , m_globalSensor(robot)
        , m_maxAbsFminusC(0)
        , m_visComponent(m_robot.addComponent<CControllerVis>())
        , m_indicator(3.14, 20, 255, 0, 0, 255)
    {
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

        vector<pair<double, double>> otherRobotIntervals = m_globalSensor.getOtherRobotIntervals(m_world, m_robot, m_config.fieldOfView, m_config.maxSensingDistance);

        bool puckValid = false;
        double puckValue = m_globalSensor.getExtremeGridValue(m_world, m_robot, "red_puck", true, 0, 1, otherRobotIntervals, m_config.fieldOfView, m_config.maxSensingDistance, puckValid);

        m_targetThreshold = 1.0;
        if (puckValid) {
            //m_targetThreshold = fmax(0.01, puckValue);
            m_targetThreshold = puckValue;
            //cerr << "m_targetThreshold: " << m_targetThreshold << endl;
        }

        double epsilon = 0;
        if (m_targetThreshold != 0)
            epsilon = (fmin(forward, 2 * m_targetThreshold) - m_targetThreshold) / m_targetThreshold;
        //cerr << "epsilon: " << epsilon << endl;

        m_maxAbsFminusC = fmax(fabs(forward - centre), m_maxAbsFminusC); 

        // Forward and angular speed, to be further specified below.
        double v = 1;
        double w = 0;

        if (forward >= 1 || centre >= 1 || right >= 1) {
            // The sensors are off the edge of the grid, or in the middle of
            // an obstacle where there is no valid reading.  Just veer right.
            w = m_config.Komega;
            m_indicator.r = 255;
            //cerr << "OFF-GRID" << endl;

        } else if (right > centre) {
            if (forward > centre)
                w = m_config.Komega;
            else
                w = -m_config.Komega;
            m_indicator.b = 255;
            //cerr << "REALIGN" << endl;
        } else {
            /**
             * Kbal should be in [0, 1] and selects the balance between these two
             * error terms:
             * 
             *  Alignment term: (forward - centre) / m_maxAbsFminusC
             *  Distance term: epsilon
             */
            w = m_config.Kbal * (forward - centre) / m_maxAbsFminusC + (1.0 - m_config.Kbal) * epsilon;

            m_indicator.g = 255;
            //cerr << "PROP" << endl;
        }

        //if (fabs(w) > 0.5) v = 0.1;

        if (otherRobotIntervals.size() > 0) v = 0.1;

        // Debug / visualization
        if (m_robot.getComponent<CControllerVis>().selected) {
            auto & visGrid = m_world->getGrid(5);
            visGrid.addContour(m_targetThreshold, m_world->getGrid(0), 1.0);
            //
            if (w > 0)
                m_indicator.angle = M_PI/2.0;
            else if (w < 0)
                m_indicator.angle = -M_PI/2.0;
            m_indicator.length = 50;
            m_robot.addComponent<CVectorIndicator>(m_indicator);
            std::stringstream ss;
            ss << m_reading.toString()
            << "epsilon: " << epsilon << endl
            << "v, w: " << v << ", " << w << endl;
            m_visComponent.msg = ss.str();
        }

        return EntityAction(v * m_config.maxForwardSpeed, w * m_config.maxAngularSpeed);
    }
};