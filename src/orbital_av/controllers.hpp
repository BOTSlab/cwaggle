#include "CWaggle.h"
#include <math.h>
#include <random>

class EntityController_OrbitalConstructionVariant : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity          m_robot;
    double          m_threshold;
    int             m_puckVariant, m_thresholdVariant, m_defaultVariant;
    size_t          m_controllerConfig;
    CVectorIndicator m_indicator;

    SensorReading   m_reading;

public:
    EntityController_OrbitalConstructionVariant(Entity robot, std::shared_ptr<World> world, double threshold,
                                                int puckVariant, int thresholdVariant, int defaultVariant,
                                                size_t controllerConfig)
        : m_world(world)
        , m_robot(robot)
        , m_threshold(threshold)
        , m_puckVariant(puckVariant)
        , m_thresholdVariant(thresholdVariant)
        , m_defaultVariant(defaultVariant)
        , m_controllerConfig(controllerConfig)
        , m_indicator(3.14, 20, 255, 0, 0, 255)
    {
    }

    virtual EntityAction getAction()
    {
        // The default indicator will be a straight-ahead white line
        m_indicator.angle = 0;
        m_indicator.length = 60;
        m_indicator.r = 255;
        m_indicator.g = 255;
        m_indicator.b = 255;
        m_indicator.a = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);

        // read the sensors and store it in m_reading
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        const double MaxAngularSpeed = 0.1;
        const double ForwardSpeed = 2;
        EntityAction left(ForwardSpeed, -MaxAngularSpeed);
        EntityAction right(ForwardSpeed, MaxAngularSpeed);
        EntityAction shallowLeft(ForwardSpeed, -0.3*MaxAngularSpeed);
        EntityAction shallowRight(ForwardSpeed, 0.3*MaxAngularSpeed);
        EntityAction slowLeft(0.25*ForwardSpeed, -MaxAngularSpeed);
        EntityAction slowRight(0.25*ForwardSpeed, MaxAngularSpeed);
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
        if (C > 0.5) {
            if (m_reading.leftRobots > 0)
                // Good for avoiding oscillations between avoiding border/robots, but maybe
                // not so good for when congestion arises at the periphery of the structure.
                return slowRight;
        } else {
            if (m_reading.rightRobots > 0)
                return slowLeft;
        }
        */

        
        bool robotRight = m_reading.rightRobots > 0;
        bool robotLeft = m_reading.leftRobots > 0;

        bool pucksPerceived = m_reading.leftPucks > 0;

//        if (m_reading.rightRobots > 0 && m_reading.leftRobots > 0)
//            return slowStraight;

        if (m_reading.rightRobots > 0)
            return slowStraight;

        if (m_reading.leftRobots > 0)
            pucksPerceived = false;

        if ((m_puckVariant & ordering) && pucksPerceived) {
//            return (!robotLeft) ? left : right; 
            return left; 
        }
        if (m_thresholdVariant & ordering) {
            if (C < m_threshold)
                return shallowLeft;
//                return (!robotLeft) ? shallowLeft : shallowRight;
            else
                return shallowRight;
//                return (!robotRight) ? shallowRight : shallowLeft;
        }
        if (m_defaultVariant & ordering)
//            return (!robotRight) ? right : left;
            return right;
        else
//            return (!robotLeft) ? left : right;
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

        if (m_reading.leftObstacle > 0)
        {
            m_previousAction = EntityAction(ForwardSpeed, MaxAngularSpeed);
            return m_previousAction;
        }

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

