#pragma once

#include "CWaggle.h"
#include <math.h>
#include <random>
#include "configs.hpp"

class EntityController_OrbitalConstructionVariant : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity          m_robot;
    double          m_threshold;
    ControllerConfig m_config;
    CVectorIndicator m_indicator;

    SensorReading   m_reading;

public:
    EntityController_OrbitalConstructionVariant(Entity robot, std::shared_ptr<World> world, double threshold,
                                                ControllerConfig config)
        : m_world(world)
        , m_robot(robot)
        , m_threshold(threshold)
        , m_config(config)
        , m_indicator(3.14, 20, 255, 0, 0, 255)
    {
    }

    void indicateNothing() {
        m_indicator.angle = 0;
        m_indicator.length = 0;
        m_indicator.r = 0;
        m_indicator.g = 0;
        m_indicator.b = 0;
        m_indicator.a = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);
    }
    void indicateLeft() {
        m_indicator.angle = -M_PI/2.0;
        m_indicator.length = 100;
        m_indicator.g = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);
    }

    void indicateRight() {
        m_indicator.angle = M_PI/2.0;
        m_indicator.length = 100;
        m_indicator.r = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);
    }

    void indicateSlowStraight() {
        m_indicator.angle = 0;
        m_indicator.length = 100;
        m_indicator.b = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);
    }

    void indicateStraight() {
        m_indicator.angle = 0;
        m_indicator.length = 100;
        m_indicator.r = 255;
        m_indicator.g = 255;
        m_indicator.b = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);
    }


    virtual EntityAction getAction() {
        indicateNothing();

        // read the sensors and store it in m_reading
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        const double MaxAngularSpeed = 0.1;
        const double ForwardSpeed = 2;
        const double ShallowFactor = 1.0;
        EntityAction straight(ForwardSpeed, 0);
        EntityAction left(ForwardSpeed, -MaxAngularSpeed);
        EntityAction right(ForwardSpeed, MaxAngularSpeed);
        EntityAction shallowLeft(ForwardSpeed, -ShallowFactor*MaxAngularSpeed);
        EntityAction shallowRight(ForwardSpeed, ShallowFactor*MaxAngularSpeed);
        EntityAction slowSlightLeft(0.25*ForwardSpeed, -0.5*MaxAngularSpeed);
        EntityAction slowStraight(0.25*ForwardSpeed, 0);

        size_t type = m_robot.getComponent<CRobotType>().type;

        double L = m_reading.leftNest;
        double C = m_reading.midNest;
        double R = m_reading.rightNest;

        // Determine the current ordering of scalar field sensors.
        unsigned int ordering = 0;
        if (L >= C && C >= R) {
            ordering = 0b000001;
        } else if (L >= R && R >= C) {
            ordering = 0b000010;            
        } else if (C >= L && L >= R) {
            ordering = 0b000100;            
        } else if (C >= R && R >= L) {
            ordering = 0b001000;            
        } else if (R >= L && L >= C) {
            ordering = 0b010000;            
        } else if (R >= C && C >= L) {
            ordering = 0b100000;            
        }

        /*
        bool robotLeft = m_reading.leftRobots > 0;
        bool robotRight = m_reading.rightRobots > 0;
        if (C > 0.5) {
            if ((m_config.leftRobotHiVariant & ordering) && robotLeft
                &&
                (m_config.rightRobotHiVariant & ordering) && robotRight)
                return slowStraight;
            if ((m_config.leftRobotHiVariant & ordering) && robotLeft)
                return right;
            if ((m_config.rightRobotHiVariant & ordering) && robotRight)
                return left;
        } else {
            if ((m_config.leftRobotLoVariant & ordering) && robotLeft
                &&
                (m_config.rightRobotLoVariant & ordering) && robotRight)
                return slowStraight;
            if ((m_config.leftRobotLoVariant & ordering) && robotLeft)
                return right;
            if ((m_config.rightRobotLoVariant & ordering) && robotRight)
                return left;
        }
        if (C > 0.5) {
            if (robotLeft) {
                indicateRight();
                return right;
            }
        } else {
            if (robotLeft && robotRight) {
                indicateStraight();
                return slowStraight;
            }
            if (robotLeft) {
                indicateLeft();
                return left;
            }
        }
        */

        int n = m_reading.image.size();
//        if (C > 0.5) {
            // Find the biggest gap between robots in the robot-only image.
            int longestI = 0, longestJ = -1;
            for (int i=0; i < n; i++)
            {
                if (!m_reading.image[i]) {
                    for (int j = i; !m_reading.image[j] && j < n; j++) {
                        if (j - i > longestJ - longestI) {
                            longestI = i;
                            longestJ = j;
                        }
                    }
                }
            }

            if (longestI != 0 || longestJ != n-1)
            {   // There is at least one robot in view...
                if (longestI == 0) {
                    indicateLeft();
                    return left;
                } if (longestJ == n-1) {
                    indicateRight();
                    return right;
                }

                // Considering the centre of the gap to be the average of 
                // its two ends, longestI and longestJ.  Turn left/right to move
                // this point towards the centre of the image.
                int gapCentre = (longestI + longestJ)/2;
                int error = n/2 - gapCentre;
                if (error > 0 && (63 & ordering)) {
                    indicateLeft();
                    return left;
                } if (error < 0) {
                    indicateRight();
                    return right;
                }
                indicateSlowStraight();
                return slowStraight;
            }

//        } else
//        {
//            // Only react to another robot if it is in the left field of view.
//            for (int i=0; i < n/2; i++)
//            {
//                if (m_reading.image[i]) {
//                    indicateSlowStraight();
//                    return slowSlightLeft;
//                }
//            }
//        }
        
        bool pucksPerceived = m_reading.leftPucks > 0;
        if ((m_config.puckVariant & ordering) && pucksPerceived) {
            return left; 
        }
        if (m_config.thresholdVariant & ordering) {
            if (C < m_threshold)
                return shallowLeft;
            else
                return shallowRight;
        }
        if (m_config.defaultVariant & ordering)
            return right;
        else
            return left;
    }
};

/*
class EntityController_OrbitalConstruction2 : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity          m_robot;
    double          m_threshold;
    CVectorIndicator m_indicator;

    double          m_timeSinceLastPuckSeen;

    SensorReading   m_reading;

public:
    EntityController_OrbitalConstruction2(Entity robot, std::shared_ptr<World> world,
                                          double threshold)
        : m_world(world)
        , m_robot(robot)
        , m_threshold(threshold)
        , m_indicator(3.14, 20, 255, 0, 0, 255)
        , m_timeSinceLastPuckSeen(0)
    {
    }

    virtual EntityAction getAction()
    {
        //m_threshold *= 0.99999;

        // The default indicator will be a straight-ahead white line
        m_indicator.angle = 0;
        m_indicator.length = 2;
        m_indicator.r = 255;
        m_indicator.g = 255;
        m_indicator.b = 255;
        m_indicator.a = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);

        // read the sensors and store it in m_reading
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        const double MaxAngularSpeed = 0.3;
        const double ForwardSpeed = 2;

        if (m_reading.leftObstacle > 0)
        {
            m_previousAction = EntityAction(ForwardSpeed, MaxAngularSpeed);
            return m_previousAction;
        }

//        size_t type = m_robot.getComponent<CRobotType>().type;
//size_t type = 0;
//        bool innie = type == 1;
//bool innie = m_threshold[0] < 0.6;

        double threshold = m_threshold;// + normalDist(generator);

        if (m_reading.leftNest >= m_reading.midNest && m_reading.midNest >= m_reading.rightNest)
        {
            // The gradient is in the desired orientation with the highest
            // sensed value to the left, then the centre value in the middle,
            // followed by the lowest on the right.

            // These conditions steer in (for an innie) and out (for an outie)
            // to nudge a puck inwards or outwards.

            if (m_reading.leftPucks > 0)
            {   
                // Curve out (left) to gather this puck.
                m_previousAction = EntityAction(ForwardSpeed, -1.5 * MaxAngularSpeed);
                return m_previousAction;
            }
            else if (m_reading.rightPucks > 0
// ONLY EJECT PUCK IF THE OPPOSITE GRID SENSOR ARRAY INDICATES THAT THE OPPOSITE GOAL IS TO THE RIGHT.
&& m_reading.rightOppNest >= m_reading.midOppNest && m_reading.midOppNest >= m_reading.leftOppNest
                )
            {
                m_previousAction = EntityAction(ForwardSpeed, MaxAngularSpeed);
                return m_previousAction;
            }

            // We now act to maintain the centre value at the desired isoline.
            if (m_reading.midNest < threshold)
            {
                m_previousAction = EntityAction(ForwardSpeed, -0.3 * MaxAngularSpeed);
                return m_previousAction;
            }
            else
            {
                m_previousAction = EntityAction(ForwardSpeed, 0.3 * MaxAngularSpeed);
                return m_previousAction;
            }
        }
        else if (m_reading.midNest >= m_reading.rightNest && m_reading.midNest >= m_reading.leftNest)
        {
            m_previousAction = EntityAction(ForwardSpeed, MaxAngularSpeed);
            return m_previousAction;
        }
        else
        {
            m_previousAction = EntityAction(ForwardSpeed, -MaxAngularSpeed);
            return m_previousAction;
        }
    }
};
*/


class EntityController_OrbitalConstruction : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity          m_robot;
    SensorReading   m_reading;
    double          m_threshold[2] = { .9, 0.6 };

public:

    EntityController_OrbitalConstruction(Entity robot, std::shared_ptr<World> world)
        : m_world(world)
        , m_robot(robot)
    {
    }

    virtual EntityAction getAction()
    {
        // read the sensors and store it in m_reading
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        const double MaxAngularSpeed = 0.3;
        const double ForwardSpeed = 2;

        
//        if (m_reading.leftObstacle > 0)
//        {
//            m_previousAction = EntityAction(ForwardSpeed, MaxAngularSpeed);
//            return m_previousAction;
//        }

        size_t type = m_robot.getComponent<CRobotType>().type;
        bool innie = type == 1;

        if (m_reading.rightNest >= m_reading.midNest && m_reading.midNest >= m_reading.leftNest)
        {
            // The gradient is in the desired orientation with the highest
            // sensed value to the right, then the centre value in the middle,
            // followed by the lowest on the left.

            // These conditions steer in (for an innie) and out (for an outie)
            // to nudge a puck inwards or outwards.
            if (innie && m_reading.rightPucks > 0)
            {
                m_previousAction = EntityAction(ForwardSpeed, MaxAngularSpeed);
                return m_previousAction;
            }
            else if (!innie && m_reading.leftPucks > 0)
            {
                m_previousAction = EntityAction(ForwardSpeed, -MaxAngularSpeed);
                return m_previousAction;
            }

            // We now act to maintain the centre value at the desired isoline.
            if (m_reading.midNest < m_threshold[type])
            {
                m_previousAction = EntityAction(ForwardSpeed, 0.3 * MaxAngularSpeed);
                return m_previousAction;
            }
            else
            {
                m_previousAction = EntityAction(ForwardSpeed, -0.3 * MaxAngularSpeed);
                return m_previousAction;
            }
        }
        else if (m_reading.midNest >= m_reading.rightNest && m_reading.midNest >= m_reading.leftNest)
        {
            // We are heading uphill of the gradient, turn left to return to a
            // clockwise orbit.
            m_previousAction = EntityAction(ForwardSpeed, -MaxAngularSpeed);
            return m_previousAction;
        }
        else
        {
            // We are heading downhill, turn right to return to clockwise orbit.
            m_previousAction = EntityAction(ForwardSpeed, MaxAngularSpeed);
            return m_previousAction;
        }
    }
};

