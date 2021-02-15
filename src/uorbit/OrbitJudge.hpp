#pragma once

#include "CWaggle.h"
#include "Intersect.h"
#include <math.h>
#include <random>

using namespace std;

constexpr double SQRT2 = 1.41421;

/**
 * This class assigns scores to potential orbit circles.
 */
class OrbitJudge {
    std::shared_ptr<World> m_world;
    ExperimentConfig m_config;
    double m_minPossibleScore, m_maxPossibleScore;
    double m_lowestTheoreticalScore;

public:
    OrbitJudge(std::shared_ptr<World> world, ExperimentConfig config)
        : m_world(world)
        , m_config(config)
        , m_minPossibleScore(0)
        , m_maxPossibleScore(0)
    {
        m_lowestTheoreticalScore =-(int)(m_config.nRedPucks + m_config.nGreenPucks);
    }

    double getScore(Entity robot, tuple<double, double, double> t, Vec2 oldCircleCentre) {
        // Circle's centre and radius
        Vec2 circleCentre{std::get<0>(t), std::get<1>(t)};
        double circleRadius = std::get<2>(t);

        // Does a circle with the radius of the current circle + the robot's
        // radius intersect any line bodies?  If so, just give it a low score.
        for (auto lineEntity : m_world->getEntities("line")) {
            auto & line = lineEntity.getComponent<CLineBody>();
            if (Intersect::checkCircleSegmentIntersection(line.s, line.e, circleCentre, circleRadius + m_config.robotRadius))
                return m_lowestTheoreticalScore;
        }

        // Does this circle (enlarged by the robot's radius) contain any other
        // robots?
        for (auto e : m_world->getEntities("robot")) {
            if (e.id() == robot.id()) continue;

            auto& otherTerritory = e.getComponent<CTerritory>();
            double d = circleCentre.dist(otherTerritory.centre);
            if (d < circleRadius + otherTerritory.radius + 2*m_config.robotRadius)
                return m_lowestTheoreticalScore;
        }

        double score, redScore, greenScore;
        bool validRedScore = scoreForPuckType(redScore, robot, t, "red_puck", m_config.redGoalX, m_config.redGoalY, m_config.redGoalRadius, 0, 1);
        bool validGreenScore = scoreForPuckType(greenScore, robot, t, "green_puck", m_config.greenGoalX, m_config.greenGoalY, m_config.greenGoalRadius, 2, 3);
        if (validRedScore && validGreenScore) {
            score = redScore + greenScore;

            // Multiply the score by a decaying exponential factor which is a function of distance.
            // from the current centre to the given centre.
            /*
            double tau = 20;
            double d = oldCircleCentre.dist(circleCentre);
            score *= exp(-d / tau);
            */

            // Modulate the score to account for the circumference.
            //score /= 2 * M_PI * circleRadius;

            // NOT IMPORTANT: Just trying allay nagging fears about the score.
            m_minPossibleScore = min(m_minPossibleScore, score);
            m_maxPossibleScore = max(m_maxPossibleScore, score);
        } else {
            score = m_lowestTheoreticalScore;
        }

//cerr << "m_lowestTheoreticalScore: " << m_lowestTheoreticalScore << endl;
//cerr << "m_minPossibleScore: " << m_minPossibleScore << endl;
//cerr << "m_maxPossibleScore: " << m_maxPossibleScore << endl;

        return score;
    }

    bool scoreForPuckType(double &score, Entity robot, tuple<double, double, double> t, string puck_type, double goalX, double goalY, double goalRadius, int desiredXIndex, int desiredYIndex) {
        bool clockwise = true;

        // Circle's centre and radius
        Vec2 circleCentre{std::get<0>(t), std::get<1>(t)};
        double circleRadius = std::get<2>(t);

        auto & desiredXGrid = m_world->getGrid(desiredXIndex);
        auto & desiredYGrid = m_world->getGrid(desiredYIndex);

        // Does this circle intersect the goal's circle?  If so, assign a low score.
        double dummy1, dummy2, dummy3, dummy4;
        int intersection = Intersect::circleCircleIntersection(circleCentre.x, circleCentre.y, circleRadius,
                                    goalX, goalY, goalRadius + m_config.robotRadius,
                                    &dummy1, &dummy2, &dummy3, &dummy4);
        if (intersection != 0)
            return false;

        score = 0;

        for (auto e : m_world->getEntities(puck_type)) {
            Vec2 puckPos = e.getComponent<CTransform>().p;
            auto & puckBody = e.getComponent<CCircleBody>();

            // d is the distance from the circle's centre to the puck's centre
            double d = circleCentre.dist(puckPos);

            if ((d + puckBody.r < circleRadius - m_config.robotRadius) ||
                (d - puckBody.r > circleRadius + m_config.robotRadius)) {
                // Puck is not within the disk-shaped region where it would
                // collide with a robot travelling around this circle.
                continue;
            }

            // We determine the putative position for a robot orbiting about
            // this circle by solving for the intersection of the orbit circle
            // and a circle centred at the puck's position but with a radius
            // of puckRadius + m_config.robotRadius. 
            double i1x, i1y, i2x, i2y;
            int intersection = Intersect::circleCircleIntersection(circleCentre.x, circleCentre.y, circleRadius,
                                        puckPos.x, puckPos.y, puckBody.r,
                                        &i1x, &i1y, &i2x, &i2y);
            if (intersection == 0)
                continue;

            // Determine which of (i1x, i1y) and (i2x, i2y) corresponds to the
            // robot's position on the orbit circle given a CW or CCW orbit.
            Vec2 robotPos;
            int orient = Intersect::orientation(Vec2{i1x, i1y}, circleCentre, puckPos);
            if (orient == 0)
                // This shouldn't be possible.  We just won't assign a score here.
                continue;
            if (clockwise && orient == 1) { // SHOULD ORIENT BE == 2 ???
                robotPos.x = i1x;
                robotPos.y = i1y;
            } else {
                robotPos.x = i2x;
                robotPos.y = i2y;
            }

            // Now given the robot and puck positions (and the fact that they
            // are touching) figure out the direction the puck would move
            // assuming a perfectly elastic collision.  Following:
            // https://www.vobarian.com/collisions/2dcollisions2.pdf

            // Compute the unit normal vector (normal to the collision between
            // robot and puck).
            Vec2 normal = (robotPos - puckPos).normalize();;
            //Vec2 tangent{-normal.y, normal.x};

            // v1 is the putative robot speed.  We don't bother to work out its
            // actual magnitude because that will just be a scaling factor in 
            // the score.
            Vec2 centreToRobot = robotPos - circleCentre;
            Vec2 v1{-(robotPos.y - circleCentre.y), 
                    robotPos.x - circleCentre.x};
            v1 = v1.normalize();
            if (!clockwise)
                v1 *= -1;

            // Project the robot's speed into its normal and tangential components.
            double v1n = normal.dot(v1);
            // double v1t = tangent.dot(v1);

            // The normal component of the puck's velocity post-collision.  Note
            // that we assume the puck is initially at rest.  We are also assuming
            // here that the puck's mass is negligible, otherwise we would have
            // v2n = 2 m1 v1n / (m1 + m2)
            double v2n = 2 * v1n;

            // The puck's velocity post-collision.
            Vec2 v2 = normal * v2n;

CVectorIndicator vi;
vi.angle = atan2(v2.y, v2.x);
vi.length = 100;
vi.r = 255;
vi.g = 255;
vi.b = 255;
vi.a = 255;
e.addComponent<CVectorIndicator>(vi);

            // Finally, increment the score by the dot product of the puck's
            // post-collision motion and the puck's desired motion.
            size_t gX = (size_t)round(puckPos.x);
            size_t gY = (size_t)round(puckPos.y);
            Vec2 desired{
                SQRT2 * (desiredXGrid.get(gX, gY) - 0.5), 
                SQRT2 * (desiredYGrid.get(gX, gY) - 0.5)
            };
            score += v2.dot(desired);
            //score += max(score, v2.dot(desired));
        }

auto & scoreGrid = m_world->getGrid(4);
size_t gX = (size_t)round(circleCentre.x);
size_t gY = (size_t)round(circleCentre.y);
scoreGrid.set(gX, gY, score);

        return true;
    }
};