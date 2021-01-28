#pragma once

#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "CWaggle.h"
#include "GUI.hpp"

#include "MyEval.hpp"
#include "configs.hpp"
#include "controllers.hpp"
#include "worlds.hpp"
#include "SpeedManager.hpp"

using namespace std;

class MyExperiment {
    ExperimentConfig m_config;
    ControllerConfig m_ctrlConfig;
    int m_trialIndex;

    shared_ptr<GUI> m_gui;
    shared_ptr<Simulator> m_sim;
    shared_ptr<World> m_world;

    size_t m_simulationSteps = 0;
    double m_simulationTime = 0;
    Timer m_simTimer;
    ofstream m_fout;

    stringstream m_status;
    double m_eval;

    default_random_engine m_rng;
    bool m_aborted;

    SpeedManager m_speedManager;

public:
    MyExperiment(ExperimentConfig config, ControllerConfig ctrlConfig, int trialIndex, int rngSeed)
        : m_config(config)
        , m_ctrlConfig(ctrlConfig)
        , m_trialIndex(trialIndex)
        , m_rng(rngSeed)
        , m_aborted(false)
        , m_speedManager((int) config.renderSteps, config.simTimeStep)
    {
        //cout << "rngSeed: " << rngSeed << endl;
        if (m_config.writePlotSkip) {
            stringstream plotFilename;
            plotFilename << m_config.plotFilenameBase << "_" << trialIndex << ".dat";
            m_fout = ofstream(plotFilename.str());
        }

        resetSimulator();
    }

    void doSimulationStep()
    {
        if (m_config.writePlotSkip && m_simulationSteps % m_config.writePlotSkip == 0)
            writeToFile();

        ++m_simulationSteps;

        if (!m_gui && (m_simulationSteps % 10000 == 0)) {
            cout << "Simulation Step: " << m_simulationSteps << "\n";
        }

        // control robots that have controllers
        for (auto& robot : m_sim->getWorld()->getEntities("robot")) {
            if (!robot.hasComponent<CController>()) {
                continue;
            }

            //if (m_speedManager.simTimeStep() > 0) {
                // get the action that should be done for this entity
                EntityAction action = robot.getComponent<CController>().controller->getAction();

                // have the action apply its effects to the entity
                action.doAction(robot, m_speedManager.simTimeStep());
            //}
        }

        // call the world physics simulation update
        // parameter = how much sim time should pass (default 1.0)
        m_sim->update(m_speedManager.simTimeStep());
    }

    void run()
    {
        bool running = true;
        while (running) {
            m_eval = LassoEval::PuckGridValues(m_world, "red_puck", 2, false);
            //cerr << "m_eval: " << m_eval << endl;
            if (isnan(m_eval)) {
                cerr << "nan evaluation encountered!\n";
                m_aborted = true;
                break;
            }

            m_simTimer.start();
            for (size_t i = 0; i < m_speedManager.renderSteps(); i++) {
                if (m_config.maxTimeSteps > 0 && m_simulationSteps >= m_config.maxTimeSteps) {
                    running = false;
                }
/*
double BEFORE = LassoEval::NanPucks(m_world, "red_puck");
for (auto e1 : m_world->getEntities("red_puck"))
{
    assert(e1.hasComponent<CTransform>());
    Vec2 pos = e1.getComponent<CTransform>().p;

    if (e1.id() == 368) {
        cerr << "id: " << e1.id() << endl;
        cerr << "pos.x: " << pos.x << endl;
        cerr << "pos.y: " << pos.y << endl;
    }
}
*/

                doSimulationStep();

/*
double AFTER = LassoEval::NanPucks(m_world, "red_puck");
if (BEFORE == 0 && AFTER > 0) {
cerr << "NAN" << endl;
for (auto e1 : m_world->getEntities("red_puck"))
{
    assert(e1.hasComponent<CTransform>());
    Vec2 pos = e1.getComponent<CTransform>().p;

    if (isnan(pos.x) || isnan(pos.y)) {
        cerr << "step: " << m_simulationSteps << endl;
        cerr << "id: " << e1.id() << endl;
        cerr << "pos.x: " << pos.x << endl;
        cerr << "pos.y: " << pos.y << endl;
        exit(0);
    }
}
}
*/

            }
            m_simulationTime += m_simTimer.getElapsedTimeInMilliSec();

            if (m_gui) {
                m_status = stringstream();
                m_status << "Step: " << m_simulationSteps << endl;
                m_status << "Eval: " << m_eval << endl;

                for (auto robot : m_sim->getWorld()->getEntities("robot"))
                {
//m_gui->clearContours();
//auto base = robot.getComponent<CController>().controller;
//auto controller = dynamic_pointer_cast<EntityController_Lasso>(base);
//m_gui->addContour(0.0, 255, 0, 0);
//m_gui->addContour(controller->m_targetThreshold, 200, 200, 200);

                    if (!robot.hasComponent<CControllerVis>()) { continue; }
                    auto vis = robot.getComponent<CControllerVis>();

                    if (vis.selected) {
                        m_status << vis.msg;
                    }
                }

// For visualization
m_gui->updateGridImage(3, true, false, false);
m_gui->updateGridImage(4, false, true, false);
m_gui->updateGridImage(5, false, false, true);
m_sim->getWorld()->getGrid(3).setAll(0);
m_sim->getWorld()->getGrid(4).setAll(0);
m_sim->getWorld()->getGrid(5).setAll(0);

                m_gui->setStatus(m_status.str());

                // draw gui
                m_gui->update();

                if (m_config.captureScreenshots) {
                    stringstream filename;
                    filename << m_config.screenshotFilenameBase << m_simulationSteps << ".png";
                    m_gui->saveScreenshot(filename.str());
                }
            }

            // RESET SIMULATOR IF DESIRED
            // resetSimulator();
        }

        if (m_gui) {
            m_gui->close();
            m_gui = NULL;
        }
    }

    void printResults()
    {
        /*
        stringstream ss;
        ss << "gnuplot/results_form_" << m_config.numRobots << "_" << m_config.maxTimeSteps << ".txt";
        cout << "Printing Results to: " << ss.str() << "\n";

        ofstream fout(ss.str());
        fout << m_eval;
        */
    }

    bool wasAborted()
    {
        return m_aborted;
    }

    double getEvaluation()
    {
        return m_eval;
    }

private:
    void addRobotControllers()
    {
        /*
        double normPuckRadius = LassoEval::BiggestGridValueDifference(m_world, 0, (int)m_config.puckRadius);
        double normRobotRadius = LassoEval::BiggestGridValueDifference(m_world, 0, (int)m_config.robotRadius);
        cout << "Computed normPuckRadius: " << normPuckRadius << endl;
        cout << "Computed normRobotRadius: " << normRobotRadius << endl;
        double normPuckRadius = 0.0588235;
        double normRobotRadius = 0.176471;
        */

        for (auto e : m_world->getEntities("robot")) {
            e.addComponent<CController>(make_shared<EntityController_Lasso>(e, m_world, m_ctrlConfig));
        }
    }

    void resetSimulator()
    {
        m_aborted = false;

        if (m_world == NULL)
            m_world = lasso_world::GetWorld(m_rng, m_config);

        // randomizeWorld(m_rng, m_config.robotRadius, m_config.puckRadius);

        m_sim = make_shared<Simulator>(m_world);

        if (m_gui) {
            m_gui->setSim(m_sim);
        } else if (m_config.gui) {
            m_gui = make_shared<GUI>(m_sim, 144);
            m_gui->setKeyboardCallback(&m_speedManager);
        }

        addRobotControllers();
    }

    void writeToFile()
    {
        /*
        for (auto & robot : m_sim->getWorld()->getEntities("robot"))
        {
            if (!robot.hasComponent<CController>()) { continue; }
            auto t = robot.getComponent<CTransform>();

            m_fout << t.p.x << " " << t.p.y << "\n";

            // Only care about robot 0 for now.
            break;
        }
        */

        m_fout << m_simulationSteps << " " << m_eval << "\n";
        m_fout.flush();
    }

};
