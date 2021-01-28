#pragma once

#include <float.h>

#include "World.hpp"
#include "Entity.hpp"
#include "Angles.h"
#include "Intersect.h"

class GlobalSensor
{
    size_t m_ownerID;
    size_t m_gridIndex = 0;
    string m_puckType;

public:
    GlobalSensor(size_t ownerID)
        : m_ownerID(ownerID)
    {
    }

    /**
     * Get the grid value for the object of the given type which has the highest
     * or lowest value, ignoring any with grid value < min or > max.  If getMax
     * is true, then it finds the highest value, otherwise it finds the lowest.  
     * If there are no objects of the given type with grid value within the
     * range [min, max] then set valid false, otherwise set it to true.
     */
    inline virtual double getExtremeGridValue(std::shared_ptr<World> world, Entity robot, string objectType, bool getMax, double min, double max, vector<pair<double, double>> forbiddenIntervals, double fieldOfView, double maxDistance, bool &valid)
    {
        bool vis = robot.getComponent<CControllerVis>().selected;

        auto & grid = world->getGrid(m_gridIndex);

        Vec2 robotPos = robot.getComponent<CTransform>().p;
        double robotAngle = robot.getComponent<CSteer>().angle;

        double extreme = getMax ? 0 : DBL_MAX;
        for (auto e : world->getEntities(objectType))
        {
            // Vis: Using the alpha channel to show pucks which pass the tests 
            // below.  Assuming none are passed, we set alpha to the default.
            if (vis) e.getComponent<CColor>().a = 255;

            Vec2 pos = e.getComponent<CTransform>().p;

            if (robotPos.dist(pos) > maxDistance)
                continue;

            double puckAngle = Angles::constrainAngle(
                atan2(pos.y - robotPos.y, pos.x - robotPos.x) - robotAngle
            );

            if (fabs(puckAngle) > fieldOfView / 2.0)
                // Puck's angle is out of range.
                continue;

            // Get object's index into the grid and the corresponding grid value, v.
            // This value will be for the centre of the object.
            size_t gX = (size_t)round(grid.width() * pos.x / world->width());
            size_t gY = (size_t)round(grid.height() * pos.y / world->height());
            double v = grid.get(gX, gY);

            if (v < min || v > max)
                // Puck's value (from scalar field) is outside desired range.
                continue;

            bool forbidden = false;
            for (pair<double, double> interval : forbiddenIntervals)
                if (v >= interval.first && v <= interval.second) {
                    forbidden = true;
                    break;
                }
            if (forbidden) 
                continue;

            // Is there a clear line of sight from the robot to this puck?
            bool intersectLines = false;
            for (auto e : world->getEntities("line")) {
                auto & line = e.getComponent<CLineBody>();

                if (Intersect::segmentsIntersect(robotPos, pos, line.s, line.e)) {
                    intersectLines = true;
                    break;
                }
            }
            if (intersectLines) 
                continue;
            
            // So we perceive this puck...
            if (vis) e.getComponent<CColor>().a = rand() % 256;

            // ...but is it the highest/lowest?
            if (getMax) {
                if (v > extreme) extreme = v;
            } else {
                if (v < extreme) extreme = v;
            }
        }

        valid = getMax ? extreme > 0 : extreme < DBL_MAX;
        return extreme;
    }

    /**
     * Get a vector of pairs, each of which represents the (min, max) values of
     * another visible robot.
     */
    inline virtual vector<pair<double, double>> getOtherRobotIntervals(std::shared_ptr<World> world, Entity robot, double fieldOfView, double maxDistance)
    {
        bool vis = robot.getComponent<CControllerVis>().selected;

        auto & grid = world->getGrid(m_gridIndex);

        Vec2 robotPos = robot.getComponent<CTransform>().p;
        double robotAngle = robot.getComponent<CSteer>().angle;

        vector<pair<double, double>> result;
        
        // Vis-only
        vector<pair<size_t, size_t>> visMinima;
        vector<pair<size_t, size_t>> visMaxima;

        for (auto e : world->getEntities("robot"))
        {
            if (robot.id() == e.id()) { continue; }

            // Vis: Using the alpha channel to show pucks which pass the tests 
            // below.  Assuming none are passed, we set alpha to the default.
            if (vis) e.getComponent<CColor>().a = 255;

            Vec2 otherPos = e.getComponent<CTransform>().p;
            auto & cb = e.getComponent<CCircleBody>();
            auto & pb = e.getComponent<CPlowBody>();
            auto & steer = e.getComponent<CSteer>();

            // Check for a direct line of sight between this robot and the other
            // robot's rear end.  We do this here by comparing the distance from
            // this robot to the other robot's centre and to its "arse". 
            double distanceToOtherRobotCentre = robotPos.dist(otherPos);
            Vec2 otherRobotArse{otherPos.x + cb.r * cos(steer.angle + M_PI),
                                otherPos.y + cb.r * sin(steer.angle + M_PI)};
            if (robotPos.dist(otherRobotArse) > distanceToOtherRobotCentre)
                continue;
            
            // Sample positions along the perimeter of this robot's body, along
            // with the tip of its plow.
            int nSamples = 16;
            vector<Vec2> samples;
            for (int i=0; i<nSamples; ++i) {
                double angle = i * 2 * M_PI / nSamples;
                Vec2 pos{otherPos.x + cb.r * cos(angle), otherPos.y + cb.r * sin(angle)};
                samples.push_back(pos);
            }
            double xProw = otherPos.x + pb.length * cos(steer.angle + pb.angle);
            double yProw = otherPos.y + pb.length * sin(steer.angle + pb.angle);
            samples.push_back(Vec2{xProw, yProw});

            // Keep track of the minimum and maximum values for this other robot
            double min = DBL_MAX;
            double max = -DBL_MAX;

            // Vis-only
            size_t visMaxGX, visMaxGY, visMinGX, visMinGY;

            for (Vec2 pos : samples) {

                if (robotPos.dist(pos) > maxDistance)
                    continue;

                double angle = Angles::constrainAngle(
                    atan2(pos.y - robotPos.y, pos.x - robotPos.x) - robotAngle
                );

                if (fabs(angle) > fieldOfView / 2.0)
                    continue;

                size_t gX = (size_t)round(grid.width() * pos.x / world->width());
                size_t gY = (size_t)round(grid.height() * pos.y / world->height());
                double v = grid.get(gX, gY);

                // Is there a clear line of sight from the robot to this point
                bool intersectLines = false;
                for (auto lineEntity : world->getEntities("line")) {
                    auto & line = lineEntity.getComponent<CLineBody>();

                    if (Intersect::segmentsIntersect(robotPos, pos, line.s, line.e)) {
                        intersectLines = true;
                        break;
                    }
                }
                if (intersectLines)
                    continue;

                if (vis) e.getComponent<CColor>().a = rand() % 256;
                
                // So we perceive this point... 
                // ...but is it the highest/lowest?
                if (v > max) max = v;
                if (v < min) min = v;

                // Vis-only
                if (vis && v == max) {
                    visMaxGX = gX;
                    visMaxGY = gY;
                }
                if (vis && v == min) {
                    visMinGX = gX;
                    visMinGY = gY;
                }
            }

            if (max > -DBL_MAX) {
                result.push_back(make_pair(min, max));

                if (vis) {
                    visMaxima.push_back(make_pair(visMaxGX, visMaxGY));
                    visMinima.push_back(make_pair(visMinGX, visMinGY));
                }
            }
        }

        // Vis-only
        if (vis) {
            auto & visGrid = world->getGrid(3);
            /*
            for (pair<size_t, size_t> pair : visMaxima)
                visGrid.set(pair.first, pair.second, 1);
            for (pair<size_t, size_t> pair : visMinima) {
                visGrid.set(pair.first, pair.second, 0.5);
            */
            for (pair<double, double> pair : result) {
                visGrid.addContour(pair.first, grid, 0.5);
                visGrid.addContour(pair.second, grid, 1.0);
            }
        }

        return result;
    }
};