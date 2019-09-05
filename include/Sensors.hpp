#pragma once

#include <memory>

#include "World.hpp"
#include "Entity.hpp"
#include "Components.hpp"

class Sensor
{
protected:

    size_t m_ownerID;         // entity that owns this sensor
    double m_angle = 0;     // angle sensor is placed w.r.t. owner heading
    double m_distance = 0;  // distance from center of owner

public:

    Sensor() {}
    Sensor(size_t ownerID, double angle, double distance)
        : m_ownerID(ownerID), m_angle(angle*3.1415926 / 180.0), m_distance(distance) { }

    inline virtual Vec2 getPosition()
    {
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double sumAngle = m_angle + Entity(m_ownerID).getComponent<CSteer>().angle;
        return pos + Vec2(m_distance * cos(sumAngle), m_distance * sin(sumAngle));
    }

    inline virtual double angle() const
    {
        return m_angle;
    }

    inline virtual double distance() const
    {
        return m_distance;
    }

    virtual double getReading(std::shared_ptr<World> world) = 0;
};


class GridSensor : public Sensor
{
    size_t m_gridIndex;    
public:

    GridSensor(size_t ownerID, size_t gridIndex, double angle, double distance)
        : Sensor(ownerID, angle, distance)
    {
        m_gridIndex = gridIndex;
    }

    inline virtual double getReading(std::shared_ptr<World> world)
    {
        if (world->getGrid(m_gridIndex).width() == 0) { return 0; }
        Vec2 sPos = getPosition();
        size_t gX = (size_t)round(world->getGrid(m_gridIndex).width()  * sPos.x / world->width());
        size_t gY = (size_t)round(world->getGrid(m_gridIndex).height() * sPos.y / world->height());
        return world->getGrid(m_gridIndex).get(gX, gY);
    }
};


class PuckSensor : public Sensor
{
    std::string m_typeName;
    double m_radius;

public:

    PuckSensor(size_t ownerID, std::string typeName, double angle, double distance, double radius)
        : Sensor(ownerID, angle, distance)
    {
        m_typeName = typeName;
        m_radius = radius;
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        double sum = 0;
        Vec2 pos = getPosition();
        for (auto puck : world->getEntities(m_typeName))
        {
            auto & t = puck.getComponent<CTransform>();
            auto & b = puck.getComponent<CCircleBody>();

            // collision with a puck
            if (t.p.distSq(pos) < (m_radius + b.r)*(m_radius + b.r))
            {
                sum += 1.0;
            }
        }
        return sum;
    }

    inline double radius() const
    {
        return m_radius;
    }

    inline void adjustAngle(double deltaAngle)
    {
        m_angle += deltaAngle;
    }
};

// https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
double constrainAngle(double x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

// Used by FancySensor below.  Represents a circle that a target object should either be within or without.
class SensingCircle {
public:
    double m_angle = 0;
    double m_distance = 0;
    double m_radius = 0;
    bool m_within = true;

    SensingCircle(double a, double d, double r, bool within) 
        : m_angle(a)
        , m_distance(d)
        , m_radius(r)
        , m_within(within)
    { }  
};

// A fancy-shaped sensor.  Considering the sets of points within circles C1 and C2.  The shape
// is C1 \cap C2 (set intersection operation).  Not inheriting from Sensor here because that class assumes
// a single circle-shaped sensing area, whereas here we need two circles to describe the shape.
class FancySensor
{
protected:

    size_t m_ownerID;       // entity that owns this sensor
public:
    std::string m_typeName;
    std::string m_sideName;
protected:
    std::vector<SensingCircle> m_circles;
    bool m_mustTouchLeft = false;
    bool m_mustTouchRight = false;

public:
    FancySensor() {}
    FancySensor(size_t ownerID, std::string typeName, std::string sideName,
                std::vector<SensingCircle> inCircles, bool mustTouchLeft, bool mustTouchRight) 
        : m_ownerID(ownerID)
        , m_typeName(typeName)
        , m_sideName(sideName)
        , m_circles(inCircles)
        , m_mustTouchLeft(mustTouchLeft)
        , m_mustTouchRight(mustTouchRight)
    { }

    inline int getNumberOfCircles()
    {
        return m_circles.size();
    }

    inline double getCircleRadius(int index)
    {
        return m_circles[index].m_radius;
    }

    inline Vec2 getCirclePosition(int index)
    {
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double sumAngle = Entity(m_ownerID).getComponent<CSteer>().angle + m_circles[index].m_angle;
        return pos + Vec2(m_circles[index].m_distance * cos(sumAngle), m_circles[index].m_distance * sin(sumAngle));
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        double sum = 0;

        std::vector<Vec2> posVector;
        for (int i=0; i<m_circles.size(); i++)
            posVector.push_back(getCirclePosition(i));

        // Position and angle of robot.  Needed only if m_mustBeLeft || m_mustBeRight.
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double theta = Entity(m_ownerID).getComponent<CSteer>().angle;

        for (auto e : world->getEntities(m_typeName))
        {
            if (e.id() == m_ownerID) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & b = e.getComponent<CCircleBody>();

            // Go through all sensing circles and determine whether the current position passes
            // the test of being within/without that circle (according to the circle's m_within).
            // Note that for testing to be without a circle, we test against a circle whose radius 
            // is reduced by one diameter of the sensed object.
            bool objectSensed = true;
            for (int i=0; i<m_circles.size(); i++) {
                SensingCircle c = m_circles[i];

                if ((c.m_within && t.p.distSq(posVector[i]) > (c.m_radius + b.r)*(c.m_radius + b.r))
                    ||
                    (!c.m_within && t.p.distSq(posVector[i]) < (c.m_radius - b.r)*(c.m_radius - b.r)))
                {
                    objectSensed = false;
                }
            }

            if (m_mustTouchLeft || m_mustTouchRight) {
                // Y-coordinate w.r.t. robot ref. frame
                double y = -sin(theta) * (t.p.x - pos.x) + cos(theta) * (t.p.y - pos.y);
                if (m_mustTouchLeft && y - b.r > 0)
                    objectSensed = false;
                if (m_mustTouchRight && y + b.r < 0)
                    objectSensed = false;
            }

            if (objectSensed)
            {
                sum += 1.0;
                //std::cout << t.p.x << "_" << t.p.y << std::endl;
                //e.addComponent<CColor>(0, rand()%255, 0, 255);
            }
        }
        return sum;
    }
};

/*class FancySensor
{
protected:

    size_t m_ownerID;       // entity that owns this sensor
    double m_c1_angle = 0;
    double m_c1_distance = 0;
    double m_c1_radius;
    double m_c2_angle = 0;
    double m_c2_distance = 0;
    double m_c2_radius = 0;
    bool m_differenceMode = false;
public:
    std::string m_typeName;
    std::string m_sideName;

    FancySensor() {}
    FancySensor(size_t ownerID, std::string typeName, std::string sideName, 
                    double angle1, double distance1, double radius1, 
                    double angle2, double distance2, double radius2, bool diffMode)
        : m_ownerID(ownerID)
        , m_typeName(typeName)
        , m_sideName(sideName)
        , m_c1_angle(angle1*3.1415926 / 180.0)
        , m_c1_distance(distance1) 
        , m_c1_radius(radius1) 
        , m_c2_angle(angle2*3.1415926 / 180.0)
        , m_c2_distance(distance2) 
        , m_c2_radius(radius2) 
        , m_differenceMode(diffMode) 
    { }

    inline Vec2 getC1Position()
    {
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double sumAngle = m_c1_angle + Entity(m_ownerID).getComponent<CSteer>().angle;
        return pos + Vec2(m_c1_distance * cos(sumAngle), m_c1_distance * sin(sumAngle));
    }

    inline Vec2 getC2Position()
    {
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double sumAngle = m_c2_angle + Entity(m_ownerID).getComponent<CSteer>().angle;
        return pos + Vec2(m_c2_distance * cos(sumAngle), m_c2_distance * sin(sumAngle));
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        if (m_differenceMode)
            return getReadingDifference(world);
        else
            return getReadingIntersection(world);
    }


    inline double getReadingDifference(std::shared_ptr<World> world)
    {
        double sum = 0;

        Vec2 pos1 = getC1Position();
        Vec2 pos2 = getC2Position();
        for (auto e : world->getEntities(m_typeName))
        {
            if (e.id() == m_ownerID) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & b = e.getComponent<CCircleBody>();

            // To sense a circlebody it must be within C1 but outside C2' where
            // C2 is a circle whose radius is smaller by one diameter than the
            // sensed object.
            if (t.p.distSq(pos1) < (m_c1_radius + b.r)*(m_c1_radius + b.r) &&
                t.p.distSq(pos2) > (m_c2_radius - b.r)*(m_c2_radius - b.r))
            {
                sum += 1.0;
            }
        }
        return sum;
    }

    inline double getReadingIntersection(std::shared_ptr<World> world)
    {
        double sum = 0;

        Vec2 pos1 = getC1Position();
        Vec2 pos2 = getC2Position();
        for (auto e : world->getEntities(m_typeName))
        {
            if (e.id() == m_ownerID) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & b = e.getComponent<CCircleBody>();

            // To sense a circlebody it must be within C1 and C2.
            if (t.p.distSq(pos1) < (m_c1_radius + b.r)*(m_c1_radius + b.r) &&
                t.p.distSq(pos2) < (m_c2_radius + b.r)*(m_c2_radius + b.r))
            {
                sum += 1.0;
            }
        }
        return sum;
    }

    inline double c1Radius() const
    {
        return m_c1_radius;
    }

    inline double c2Radius() const
    {
        return m_c2_radius;
    }
};
*/


// Detects collisions with CLineBody objects.
class ObstacleSensor : public Sensor
{
    double m_radius;

public:

    ObstacleSensor(size_t ownerID, double angle, double distance, double radius)
        : Sensor(ownerID, angle, distance)
    {
        m_radius = radius;
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        double sum = 0;
        Vec2 pos = getPosition();
        for (auto e : world->getEntities())
        {
            if (!e.hasComponent<CLineBody>()) { continue; }
            if (m_ownerID == e.id()) { continue; }

            auto & b = e.getComponent<CLineBody>();

            double lineX1 = b.e.x - b.s.x;
            double lineY1 = b.e.y - b.s.y;
            double lineX2 = pos.x - b.s.x;
            double lineY2 = pos.y - b.s.y;

            double edgeLength = lineX1 * lineX1 + lineY1 * lineY1;
            double dotProd = lineX1 * lineX2 + lineY1 * lineY2;
            double t = std::max(0.0, std::min(edgeLength, dotProd)) / edgeLength;

            // find the closest point on the line to the sensor and the distance to it
            Vec2 closestPoint(b.s.x + t * lineX1, b.s.y + t * lineY1);
            double distance = closestPoint.dist(pos);

            // pretend the closest point on the line is a circle and check collision
            // calculate the overlap between the circle and that fake circle
            double overlap = m_radius + b.r - distance;
            if (overlap > 0) {
                sum += 1.0;                
            } 
        }
        return sum;
    }

    inline double radius() const
    {
        return m_radius;
    }
};


class RobotSensor : public Sensor
{
    double m_radius;

public:

    RobotSensor(size_t ownerID, double angle, double distance, double radius)
        : Sensor(ownerID, angle, distance)
    {
        m_radius = radius;
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        double sum = 0;
        Vec2 pos = getPosition();
        for (auto e : world->getEntities())
        {
            if (!e.hasComponent<CSteer>()) { continue; }
            if (m_ownerID == e.id()) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & b = e.getComponent<CCircleBody>();

            // collision with other robot
            if (t.p.distSq(pos) < (m_radius + b.r)*(m_radius + b.r))
            {
                sum += 1.0;
            }
        }
        return sum;
    }

    inline double radius() const
    {
        return m_radius;
    }
};
