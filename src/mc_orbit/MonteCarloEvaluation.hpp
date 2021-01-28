#pragma once

#include "EpisodeRecord.hpp"

class MonteCarloEvaluation {
private:
    size_t m_width, m_height;

    // This is the value function itself, which we want to estimate.
    ValueGrid m_valueGrid;

    // The number of times each state has been visited.
    ValueGrid m_totalVisitGrid;

    // The total return accumulated for each state.
    ValueGrid m_totalReturnGrid;

    // If every_visit == true then its Every-Visit MC.  Otherwise its First-Visit MC
    bool m_everyVisit = true;

    double m_gamma = 0.9;

public:
    MonteCarloEvaluation(size_t width, size_t height)
        : m_width(width)
        , m_height(height)
        , m_valueGrid(m_width, m_height, 0)
        , m_totalVisitGrid(m_width, m_height, 0)
        , m_totalReturnGrid(m_width, m_height, 0)
    {
    }

    void incorporateEpisode(shared_ptr<EpisodeRecord> episode) 
    {
        // This is neccessary for Firt-Visit MC to mark whether each state
        // has been visited this episode.
        ValueGrid visited(m_width, m_height, 0);

        for(auto it : episode->puckPaths) {
            // This is the path for one puck.
            vector<tuple<size_t, size_t, double>>& path = it.second;
            int n = path.size();

            // Calculate the return for each instant captured along the path.
            vector<double> returns(n);
            for (int t=0; t < n; ++t) {
                auto [ x, y, reward ] = path[t];
                double G = reward;

                for (int t_limit = t; t_limit < n; ++t_limit) {
                    auto [ x, y, reward ] = path[t_limit];
                    G += pow(m_gamma, t_limit - t) * reward;
                }

                returns[t] = G;
            }

            for (int t=0; t < n; ++t) {
                // (x, y) is the state we are evaluating
                auto [ x, y, reward ] = path[t];
//std::cerr << "t: " << t << std::endl;
//std::cerr << "n: " << n << std::endl;
//std::cerr << "(x, y): " << x << ", " << y << std::endl;
                if (m_everyVisit || visited.get(x, y) == 0) {

                    m_totalVisitGrid.set(x, y, m_totalVisitGrid.get(x, y) + 1);
                    m_totalReturnGrid.set(x, y, m_totalReturnGrid.get(x, y) + returns[t]);

                    m_valueGrid.set(x, y, m_totalReturnGrid.get(x, y) / m_totalVisitGrid.get(x, y));
                }
                visited.set(x, y, 1);
            }
        }

        m_valueGrid.saveToFile("values.png");
    }
};