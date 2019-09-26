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

// A fancy-shaped sensor.  Considering the sets of points within (or without) the given vector of SensingCirlces.
class FancySensor
{
protected:

    size_t m_ownerID;       // entity that owns this sensor
public:
    std::string m_typeName;
    std::string m_sideName;
    std::vector<SensingCircle> m_circles;
protected:
    bool m_mustTouchLeft = false;
    bool m_mustTouchRight = false;
    double m_leftCentreThreshold, m_rightCentreThreshold;

public:
    FancySensor() {}
    FancySensor(size_t ownerID, std::string typeName, std::string sideName,
                std::vector<SensingCircle> inCircles, bool mustTouchLeft, bool mustTouchRight,
                double leftCentreThreshold, double rightCentreThreshold) 
        : m_ownerID(ownerID)
        , m_typeName(typeName)
        , m_sideName(sideName)
        , m_circles(inCircles)
        , m_mustTouchLeft(mustTouchLeft)
        , m_mustTouchRight(mustTouchRight)
        , m_leftCentreThreshold(leftCentreThreshold)
        , m_rightCentreThreshold(rightCentreThreshold)
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
                if (m_mustTouchLeft && y - b.r > m_leftCentreThreshold)
                    objectSensed = false;
                if (m_mustTouchRight && y + b.r < -m_rightCentreThreshold)
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

// Determine whether a circle and line segment intersection.  From:
// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
// E is the starting point of the segment,
// L is the end point, 
// C is the center of circle,
// r is the circle's radius.
bool checkCircleSegmentIntersection(Vec2 E, Vec2 L, Vec2 C, double r)
{
    Vec2 d = L - E;
    Vec2 f = E - C;

    float a = d.dot(d);
    float b = 2 * f.dot(d);
    float c = f.dot(f) - r * r;

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        return false;
    }
    else
    {
        // ray didn't totally miss sphere,
        // so there is a solution to
        // the equation.

        discriminant = sqrt(discriminant);

        // either solution may be on or off the ray so need to test both
        // t1 is always the smaller value, because BOTH discriminant and
        // a are nonnegative.
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        // 3x HIT cases:
        //          -o->             --|-->  |            |  --|->
        // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

        // 3x MISS cases
        //       ->  o                     o ->              | -> |
        // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

        if (t1 >= 0 && t1 <= 1)
        {
            // t1 is the intersection, and it's closer than t2
            // (since t1 uses -b - discriminant)
            // Impale, Poke
            return true;
        }

        // here t1 didn't intersect so we are either started
        // inside the sphere or completely past it
        if (t2 >= 0 && t2 <= 1)
        {
            // ExitWound
            return true;
        }

        // no intn: FallShort, Past, CompletelyInside
        return false;
    }
}

// Emulates a 1-d camera where each "pixel" corresponds to a line segment radiating out from the sensor.
// Detections for this pixel are based on comparing all entites of the appropriate type 
class PseudoCameraSensor
{
protected:

    size_t m_ownerID;       // entity that owns this sensor
public:
    std::string m_typeName;
    double m_innerRadius, m_outerRadius, m_fovAngle;
    int m_nSegments;
    std::vector<bool>  m_image;

public:
    PseudoCameraSensor() {}
    PseudoCameraSensor(size_t ownerID, std::string typeName, double innerRadius, double outerRadius, double fovAngle, int nSegments)
        : m_ownerID(ownerID)
        , m_typeName(typeName)
        , m_innerRadius(innerRadius)
        , m_outerRadius(outerRadius)
        , m_fovAngle(fovAngle)
        , m_nSegments(nSegments)
    { 
        m_image.resize(m_nSegments);
    }

    inline void getSegmentStartEnd(int i, Vec2 & p1, Vec2 & p2)
    {
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double theta = Entity(m_ownerID).getComponent<CSteer>().angle;

        double angle = theta - m_fovAngle/2.0 + i * m_fovAngle/m_nSegments;
        p1.x = pos.x + m_innerRadius * cos(angle);
        p1.y = pos.y + m_innerRadius * sin(angle);
        p2.x = pos.x + m_outerRadius * cos(angle);
        p2.y = pos.y + m_outerRadius * sin(angle);
    }

    inline std::vector<bool> getReading(std::shared_ptr<World> world)
    {
        // Position and angle of robot.
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double theta = Entity(m_ownerID).getComponent<CSteer>().angle;

        for (int i=0; i<m_nSegments; i++)
        {
            double angle = theta - m_fovAngle/2.0 + i * m_fovAngle/m_nSegments;

            // Start and end points of line segment
            Vec2 p1(pos.x + m_innerRadius * cos(angle), pos.y + m_innerRadius * sin(angle));
            Vec2 p2(pos.x + m_outerRadius * cos(angle), pos.y + m_outerRadius * sin(angle));

            // Check against all bodies of the right type.
            bool hit = false;
            for (auto e : world->getEntities(m_typeName))
            {
                if (e.id() == m_ownerID) { continue; }
                auto & t = e.getComponent<CTransform>();
                auto & b = e.getComponent<CCircleBody>();
                if (checkCircleSegmentIntersection(p1, p2, t.p, b.r)) {
                    hit = true;
                    break;
                }
            }

            m_image[i] = hit;
        }
        return m_image;
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
