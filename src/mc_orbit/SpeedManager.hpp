#pragma once

#include "KeyboardCallback.hpp"
#include <iostream>
using std::cout;
using std::endl;

/**
 * Manages the initialization and control of the renderSteps and simTimeStep
 * variables and their adjustment through the GUI.
 */
class SpeedManager : public KeyboardCallback {
private:
    int m_renderSteps;
    double m_simTimeStep;

public:
    SpeedManager(int renderSteps, double simTimeStep)
        : m_renderSteps(renderSteps)
        , m_simTimeStep(simTimeStep)
    {
    }

    void keyHandler(sf::Keyboard::Key key)
    {
        switch (key) {
        case sf::Keyboard::Up:
            increaseSpeed();
            break;
        case sf::Keyboard::Down:
            decreaseSpeed();
            break;
        case sf::Keyboard::Space:
            togglePause();
            break;

        default:
            break;
        }
    }

    int renderSteps() const { return m_renderSteps; }

    double simTimeStep() const { return m_simTimeStep; }

    void decreaseSpeed()
    {
        if (m_renderSteps > 1) {
            m_renderSteps--;
        } else if (m_renderSteps == 1 && m_simTimeStep > 0.1) {
            m_simTimeStep -= 0.1;
        }
        cout << "m_renderSteps: " << m_renderSteps << endl;
        cout << "m_simTimeStep: " << m_simTimeStep << endl;
    }

    void increaseSpeed()
    {
        if (m_renderSteps == 1 && m_simTimeStep < 1) {
            m_simTimeStep += 0.1;
        } else {
            m_renderSteps++;
        }
        cout << "m_renderSteps: " << m_renderSteps << endl;
        cout << "m_simTimeStep: " << m_simTimeStep << endl;
    }

    void togglePause()
    {
        if (m_simTimeStep > 0) {
            m_simTimeStep = 0;
            m_renderSteps = 1;
        } else {
            m_simTimeStep = 1;
            m_renderSteps = 1;
        }
        cout << "m_renderSteps: " << m_renderSteps << endl;
        cout << "m_simTimeStep: " << m_simTimeStep << endl;
    }
};