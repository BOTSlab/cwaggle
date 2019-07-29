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

// A fancy-shaped puck sensor.  Considering the sets of points within circles C1 and C2.  The shape
// is C1 - C2 (set difference operation).  Not inheriting from Sensor here because that class assumes
// a single circle-shaped sensing area, whereas here we need two circles to describe the shape.
class FancyPuckSensor
{
protected:

    size_t m_ownerID;       // entity that owns this sensor
    std::string m_typeName;
    double m_c1_angle = 0;
    double m_c1_distance = 0;
    double m_c1_radius;
    double m_c2_angle = 0;
    double m_c2_distance = 0;
    double m_c2_radius = 0;
    double m_c3_angle = 0;
    double m_c3_distance = 0;
    double m_c3_radius = 0;
public:
    bool m_intersectionMode = false;

    FancyPuckSensor() {}
    FancyPuckSensor(size_t ownerID, std::string typeName, double angle1, double distance1, double radius1, 
                    double angle2, double distance2, double radius2,
                    double angle3, double distance3, double radius3,
                    bool intersectionMode)
        : m_ownerID(ownerID)
        , m_typeName(typeName)
        , m_c1_angle(angle1*3.1415926 / 180.0)
        , m_c1_distance(distance1) 
        , m_c1_radius(radius1) 
        , m_c2_angle(angle2*3.1415926 / 180.0)
        , m_c2_distance(distance2) 
        , m_c2_radius(radius2) 
        , m_c3_angle(angle3*3.1415926 / 180.0)
        , m_c3_distance(distance3) 
        , m_c3_radius(radius3)
        , m_intersectionMode(intersectionMode)
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

    inline Vec2 getC3Position()
    {
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double sumAngle = m_c3_angle + Entity(m_ownerID).getComponent<CSteer>().angle;
        return pos + Vec2(m_c3_distance * cos(sumAngle), m_c3_distance * sin(sumAngle));
    }

    /*
    inline virtual double getReading(std::shared_ptr<World> world)
    {
        if (m_intersectionMode)
            return getReadingIntersection(world);
        else
            return getReadingDifference(world);
    }
    */

    inline double getReadingDifference(std::shared_ptr<World> world)
    {
        double sum = 0;
        //double absoluteAngle = M_PI;

        // Robot's pose (x, y, theta)
        //const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        //double theta = Entity(m_ownerID).getComponent<CSteer>().angle;
        //double x = pos.x;
        //double y = pos.y;

        Vec2 pos1 = getC1Position();
        Vec2 pos2 = getC2Position();
        Vec2 pos3 = getC3Position();
        for (auto puck : world->getEntities(m_typeName))
        {
            auto & t = puck.getComponent<CTransform>();
            auto & b = puck.getComponent<CCircleBody>();

            // To sense a puck it must be within C1 and C2 and outside C3'.
            // C3' has the same radius as C3 minus the puck's diameter.
            if (t.p.distSq(pos1) < (m_c1_radius + b.r)*(m_c1_radius + b.r) &&
                t.p.distSq(pos2) < (m_c2_radius + b.r)*(m_c2_radius + b.r) &&
                t.p.distSq(pos3) > (m_c3_radius - b.r)*(m_c3_radius - b.r))
            {
                sum += 1.0;

                /*
                // Angular width of a puck.
                double puckAngularWidth = atan(b.r / t.p.dist(pos));
                double angle = atan2(t.p.y - y, t.p.x - x) - theta - puckAngularWidth;
                angle = fabs(constrainAngle(angle));
                if (angle < absoluteAngle) {
                    absoluteAngle = angle;
                }
                */
            }
        }
        //return absoluteAngle;
        return sum;
    }

    inline double getReadingIntersection(std::shared_ptr<World> world)
    {
        double sum = 0;
        Vec2 pos1 = getC1Position();
        Vec2 pos3 = getC3Position();
        for (auto puck : world->getEntities(m_typeName))
        {
            auto & t = puck.getComponent<CTransform>();
            auto & b = puck.getComponent<CCircleBody>();

            // To sense a puck the puck must be within both C1 and C2.
            if (t.p.distSq(pos1) < (m_c1_radius + b.r)*(m_c1_radius + b.r) &&
                t.p.distSq(pos3) < (m_c3_radius + b.r)*(m_c3_radius + b.r))
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

    inline double c3Radius() const
    {
        return m_c3_radius;
    }
};



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
