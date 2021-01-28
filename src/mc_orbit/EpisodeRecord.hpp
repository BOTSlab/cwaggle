#pragma once

#include "World.hpp"
#include "Entity.hpp"
#include "Components.hpp"

#include <vector>
#include <map>
using namespace std;

struct EpisodeRecord {
    // For each entity ID, we have a vector of tuples.  Each tuple is of the
    // form (x, y, reward), and a vector of them represents the path
    // taken for the puck with this ID during this episode.
    map<size_t, vector<tuple<size_t, size_t, double>>> puckPaths;

    size_t width, height;

    EpisodeRecord(size_t w, size_t h)
        : width(w), height(h)
    {}

    void add(shared_ptr<World> world, string puckType, int rewardGridIndex)
    {
        auto & grid = world->getGrid(rewardGridIndex);

        for (auto e : world->getEntities(puckType))
        {
            Vec2 pos = e.getComponent<CTransform>().p;

            size_t id = e.id();
            /*
            if (puckPaths.find(id) == puckPaths.end()) {
                // This is the first entry for this puck.  We have to create
                // the vector to store the path.
                puckPaths[id] = vector<tuple<size_t, size_t, double>>{};
            }
            */

            // We'll use position within the grid to represent the puck's position.
            size_t gX = (size_t)round(grid.width()  * pos.x / world->width());
            size_t gY = (size_t)round(grid.height() * pos.y / world->height());
/*
cerr << "gX: " << gX << endl;
cerr << "pos.x: " << pos.x << endl;
cerr << "grid.width(): " << grid.width() << endl;
cerr << "world->width(): " << world->width() << endl;
*/

            // We'll assign a reward of +1 for reaching the goal value of 0.8
            // on the grid, and otherwise 0.
            //double reward = grid.get(gX, gY) > 0.8 ? 1 : 0;
            double reward = grid.get(gX, gY);

            // Add to the vector representing the path.
            puckPaths[id].push_back( make_tuple(gX, gY, reward) );
        }
    }
};