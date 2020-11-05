#pragma once

#include "CWaggle.h"
#include <math.h>
#include <random>
#include "configs.hpp"

using namespace std;

class EntityController_OrbitalSorting : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity          m_robot;
    ControllerConfig m_config;
    CVectorIndicator m_indicator;
    double          m_puckRadius;
    double          m_maxForwardSpeed;
    double          m_maxAngularSpeed;

    SensorReading   m_reading;

public:
    EntityController_OrbitalSorting(Entity robot, std::shared_ptr<World> world, 
                                    ControllerConfig config, double puckRadius)
        : m_world(world)
        , m_robot(robot)
        , m_indicator(3.14, 20, 255, 0, 0, 255)
        , m_puckRadius(puckRadius)
        , m_maxForwardSpeed(2)
        , m_maxAngularSpeed(0.1)
    {
    }

    virtual EntityAction getAction()
    {
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        // Values from the three grid sensors.
        double centre0 = m_reading.gridCentre0;
        double forward0 = m_reading.gridForward0;
        double right0 = m_reading.gridRight0;
        double centre1 = m_reading.gridCentre1;
        double forward1 = m_reading.gridForward1;
        double right1 = m_reading.gridRight1;

        // We use these to estimate the orientation of the scalar field's gradient.  We take the negative
        // because we want material to flow towards the centre.
        double gradAngle0 = -atan2(right0 - centre0, forward0 - centre0);
        double gradAngle1 = -atan2(right1 - centre1, forward1 - centre1);

        // Difference between gradAngle and robot's left
        // Using https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        // to compute the signed delta angle x - y, we then take the absolute value of this.
        double deltaAngle0 = -0.5 * M_PI - gradAngle0;
        double angleError0 = fabs(atan2(sin(deltaAngle0), cos(deltaAngle0)));
        double deltaAngle1 = -0.5 * M_PI - gradAngle1;
        double angleError1 = fabs(atan2(sin(deltaAngle1), cos(deltaAngle1)));

        // The amount of shift of the C2 sensing circle will be proportional to angleError 

        // Get and modify the angle of the left-puck sensor.
        auto & sensors = m_robot.getComponent<CSensorArray>();
        for (auto & sensor : sensors.fancySensors) {
            if (sensor->m_typeName == "red_puck" && sensor->m_name == "left") {
                if (angleError0 < 0.25 * M_PI) {
                    sensor->m_circles[1].m_distanceOffset = 1.5 * m_puckRadius;
                } else {
                    sensor->m_circles[1].m_distanceOffset = 0 * m_puckRadius;
                }
            }

            if (sensor->m_typeName == "green_puck" && sensor->m_name == "left") {
                if (angleError1 < 0.25 * M_PI) {
                    sensor->m_circles[1].m_distanceOffset = 1.5 * m_puckRadius;
                } else {
                    sensor->m_circles[1].m_distanceOffset = 0 * m_puckRadius;
                }
            }
        }

        // Determine if the robot is outside its territory (accessing its own position here).
        auto & t = m_robot.getComponent<CTransform>();
        auto & b = m_robot.getComponent<CCircleBody>();
        auto & territory = m_robot.getComponent<CTerritory>();
        bool outsideTerritory = (t.p.distSq(Vec2(territory.cx, territory.cy)) > (territory.radius + b.r)*(territory.radius + b.r));

        double v = m_maxForwardSpeed;
        double w;

        if (outsideTerritory)
            m_robot.addComponent<CColor>(0, 0, 255, 255);
        else
            m_robot.addComponent<CColor>(255, 0, 0, 255);


        if (m_reading.robots) {
            // Shrink/grow the territory if another robot lies within it.  Also shift the territory away from the
            // nearest robot.
            //if (territory.radius > 50)
            //    territory.radius--;

            // BAD: Directly accessing other robot positions and world dimensions!
            double w = m_world->width();
            double h = m_world->height();
            double closestDistance = w + h;
            Vec2 closestOtherRobot;
            for (auto otherRobot : m_world->getEntities("robot")) {
                auto & ot = otherRobot.getComponent<CTransform>();
                if ((t.p != ot.p) && t.p.dist(ot.p) < closestDistance) {
                    closestDistance = t.p.dist(ot.p);
                    closestOtherRobot = ot.p;
                }
            }
            Vec2 shift = t.p - closestOtherRobot;
            shift = shift.normalize();
            territory.cx += 0.5*shift.x;
            territory.cy += 0.5*shift.y;
            territory.cx = fmax(territory.cx, territory.radius);
            territory.cy = fmax(territory.cy, territory.radius);
            territory.cx = fmin(territory.cx, w - territory.radius);
            territory.cy = fmin(territory.cy, h - territory.radius);

        } else if (territory.radius < 100) {
            territory.radius++;
        }

        if (outsideTerritory) {
            return getTerritoryHomingAction();
        } else if (((m_reading.leftRedPucks || m_reading.leftGreenPucks) &&
             (m_reading.rightRedPucks || m_reading.rightGreenPucks)) /* ||
            (!m_reading.leftRedPucks && !m_reading.leftGreenPucks &&
             !m_reading.rightRedPucks && !m_reading.rightGreenPucks) */ ) {
            // Go straight to reach the periphery.
            w = 0;
        } else if (m_reading.leftRedPucks || m_reading.leftGreenPucks) {
            w = -m_maxAngularSpeed;
        } else {
            w = m_maxAngularSpeed;
        }

        return EntityAction(v, w);
    }

    EntityAction getTerritoryHomingAction() {
        auto & t = m_robot.getComponent<CTransform>();
        auto & b = m_robot.getComponent<CCircleBody>();
        auto & steer = m_robot.getComponent<CSteer>();
        auto & territory = m_robot.getComponent<CTerritory>();

        double dist_error = t.p.dist(Vec2(territory.cx, territory.cy)) - territory.radius;

        // n is a parameter which causes tighter control over the distance
        // error versus alignment with a clockwise orbit.
        double n = 0.05;

        // A shifted version of the logistic function applied to increase the speed
        // of correction in terms of distance from the desired contour.  Think of
        // it as being applied with dist_error as the "x" variable and n as a parameter
        // governing how aggressive we are in correcting distance.
        double logistic = 2.0 / (1 + exp(-n * dist_error)) - 1.0;

        // Actual angle to the territory's centre ("home") and the angle at which to maintain the
        // territory's centre to smoothly orbit inwards.
        double home_angle = atan2(territory.cy - t.p.y, territory.cx - t.p.x) - steer.angle;

        double ideal_home_angle = -(M_PI / 2.0) * (1.0 - logistic);

        // Now define the angle_error
        // Using https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        // to compute the signed delta angle x - y
        double delta_angle = ideal_home_angle - home_angle;
        double angle_error = -atan2(sin(delta_angle), cos(delta_angle));

        //cout << "\tdist_error: " << dist_error
        //     << "\thome_angle: " << home_angle 
        //     << "\tangle_error: " << angle_error << endl;

        // Forward and angular speeds in the range [-1, 1].
        double v = cos(angle_error);
        double w = sin(angle_error);

        return EntityAction(v * m_maxForwardSpeed, w * m_maxAngularSpeed);
    }
};

// THIS VERSION IS USING THE THRESHOLD-SEEKING BEHAVIOUR FROM THE ANTS PAPER.  THE PROBLEM IS THAT SOMETIMES
// IT CAN'T REACH AN OUTLYING PUCK BUT STILL BENDS OUT SEEKING IT (SIMILAR TO PAPER SUBMITTED TO IROS).
/*
class EntityController_OrbitalSorting : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity          m_robot;
    ControllerConfig m_config;
    CVectorIndicator m_indicator;
    double          m_normPuckRadius;
public:
    double          m_targetThreshold;

    SensorReading   m_reading;

    EntityController_OrbitalSorting(Entity robot, std::shared_ptr<World> world, 
                                    ControllerConfig config, double normPuckRadius)
        : m_world(world)
        , m_robot(robot)
        , m_indicator(3.14, 20, 255, 0, 0, 255)
        , m_normPuckRadius(normPuckRadius)
        , m_targetThreshold(0.25)
    {
    }

    virtual EntityAction getAction()
    {
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        // Values from the three grid sensors.
        double centre = m_reading.gridCentre;
        double forward = m_reading.gridForward;
        double right = m_reading.gridRight;

        // We use these to estimate the orientation of the scalar field's gradient.
        double grad_angle = atan2(right - centre, forward - centre);

        m_targetThreshold = fmax(0.001, m_reading.rightPucks + 1.0 * (m_reading.gridGradY - 0.5) * m_normPuckRadius);

        // We want to use proportional control to keep the robot at the orbit
        //# defined by the m_targetThreshold, while facing orthogonal to the gradient.  
        //# So we need an error signal such as this one:
        //#       dist_error = m_targetThreshold - dist
        //# But its unknown scale presents a problem.  So instead we use the 
        //# following which lies in the range [-1, 1]:
        double dist_error = (m_targetThreshold - fmin(centre, 2*m_targetThreshold)) / (m_targetThreshold);
        // One consequence of using this error signal is that the response
        //# at a distance greater than 2*m_targetThreshold is the same as at
        //# 2*m_targetThreshold.

        // But we also have to consider the current angle w.r.t. the
        // gradient angle.  Ideally, the gradient angle should be at pi/2
        // w.r.t. the robot's forward axis when dist_error == 0.  If
        // dist_error is positive (meaning we are too far from the centre)
        // then the ideal gradient angle should be in the range
        // [pi, pi/2].  Otherwise, the ideal gradient angle should be in
        // the range [0, -pi/2].  We will use dist_error to scale linearly
        // within these ranges as we compute the ideal gradient angle.

        //# n is a parameter which causes tighter control over the distance
        //# error versus alignment with the desired contour.  We modulate it 
        //# here by the time since the last contour switch using another
        //# parameter, rate_divisor which can be increased to slow down
        //# changes in n.
        //rate_divisor = 5
        //n = min(10, 1 + (self.step - self.last_contour_switch_step)//rate_divisor)
        double n = 4;

        // A shifted version of the logistic function applied to increase the speed
        // of correction in terms of distance from the desired contour.  Think of
        // it as being applied with dist_error as the "x" variable and n as a parameter
        // governing how aggressive we are in correcting distance.
        double logistic = 2.0 / (1 + exp(-n * dist_error)) - 1.0;

        //double ideal_gradient_angle = -(M_PI / 2.0) * (1.0 - pow(dist_error, n));
        //double ideal_gradient_angle = -(M_PI / 2.0) * pow(1.0 - dist_error, n);
        double ideal_gradient_angle = -(M_PI / 2.0) * (1.0 - logistic);

        //ideal_gradient_angle = pi_2 * (1 - logistic_modified(dist_error, n))
        //if self.counterclockwise:
        //    ideal_gradient_angle *= -1
        //#print("\t target_threshold, n, dist_error, ideal_gradient_angle: {:.3f}\t {} \t{:.3f} \t{:.3f}".format(self.target_threshold, n, dist_error, ideal_gradient_angle))
        //#assert(ideal_gradient_angle >= 0 and ideal_gradient_angle <= pi)

        // Now define the angle_error
        // Using https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        // to compute the signed delta angle x - y
        double delta_angle = ideal_gradient_angle - grad_angle;
        double angle_error = -atan2(sin(delta_angle), cos(delta_angle));

        //cout << "grad_angle: " << grad_angle 
        //     << "\tdist_error: " << dist_error
        //     << "\tangle_error: " << angle_error << endl;

        // Forward and angular speeds in the range [-1, 1].
        double v = cos(angle_error);
        double w = sin(angle_error);

        const double MaxForwardSpeed = 2;
        const double MaxAngularSpeed = 0.3;

        return EntityAction(v * MaxForwardSpeed, w * MaxAngularSpeed);
    }
};
*/

/*
class EntityController_OrbitalSorting : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity          m_robot;
    double          m_threshold;
    ControllerConfig m_config;
    CVectorIndicator m_indicator;
    double          m_normPuckRadius;

    SensorReading   m_reading;

public:
    EntityController_OrbitalSorting(Entity robot, std::shared_ptr<World> world, 
                                    ControllerConfig config, double normPuckRadius)
        : m_world(world)
        , m_robot(robot)
        , m_threshold(0.0)
        , m_indicator(3.14, 20, 255, 0, 0, 255)
        , m_normPuckRadius(normPuckRadius)
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

        double threshold;
        bool react_to_pucks = false;
        double react_sign = 1;
        if (m_reading.midOppNest > 0) {
            // Top-half
            threshold = m_reading.topPucks - m_normPuckRadius;
            //react_to_pucks = m_reading.topPucks > 0;
            //react_sign = 1;
            m_robot.addComponent<CColor>(255, 0, 0, 255);
        } else {
            // Bottom-half
            threshold = m_reading.bottomPucks + m_normPuckRadius;
            //react_to_pucks = m_reading.bottomPucks > 0;
            //react_sign = -1;
            m_robot.addComponent<CColor>(0, 255, 0, 255);
        }
        //threshold = fmin(threshold, 0.8);
//cout << "threshold: " << threshold << endl;

        if (m_reading.leftNest >= m_reading.midNest && m_reading.midNest >= m_reading.rightNest)
        {
            // The gradient is in the desired orientation with the highest
            // sensed value to the left, then the centre value in the middle,
            // followed by the lowest on the right.

            // These conditions steer in (for an innie) and out (for an outie)
            // to nudge a puck inwards or outwards.

            if (react_to_pucks)
            {   
                // Curve out (left) to gather this puck.
                m_previousAction = EntityAction(ForwardSpeed, react_sign * 1.5 * MaxAngularSpeed);
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