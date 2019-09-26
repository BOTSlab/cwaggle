#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <functional>

#include "CWaggle.h"
#include "GUI.hpp"

#include "configs.hpp"
#include "worlds.hpp"
#include "controllers.hpp"
#include "MyEval.hpp"

// Kludge to get the up/down arrows of the GUI to be able to modify the rendering
// speed.
int renderStepsAdjusted = 0;
double simTimeStepAdjusted = 0;
void decreaseRenderSpeed() {
    if (renderStepsAdjusted > 1) {
        renderStepsAdjusted--;
    } else if (renderStepsAdjusted == 1 && simTimeStepAdjusted > 0.1) {
        simTimeStepAdjusted -= 0.1; 
    }
    std::cout << "renderStepsAdjusted: " << renderStepsAdjusted << std::endl;
    std::cout << "simTimeStepAdjusted: " << simTimeStepAdjusted << std::endl;
}
void increaseRenderSpeed() {
    if (renderStepsAdjusted == 1 && simTimeStepAdjusted < 1) {
        simTimeStepAdjusted += 0.1;
    } else {
        renderStepsAdjusted++;        
    }
    std::cout << "renderStepsAdjusted: " << renderStepsAdjusted << std::endl;
    std::cout << "simTimeStepAdjusted: " << simTimeStepAdjusted << std::endl;
}
void togglePause() {
    if (simTimeStepAdjusted > 0) {
        simTimeStepAdjusted = 0;
        renderStepsAdjusted = 1;
    } else {
        simTimeStepAdjusted = 1;
        renderStepsAdjusted = 1;
    }
    std::cout << "renderStepsAdjusted: " << renderStepsAdjusted << std::endl;
    std::cout << "simTimeStepAdjusted: " << simTimeStepAdjusted << std::endl;
}

std::shared_ptr<World> world;

void randomizeWorld(std::default_random_engine generator, double robotRadius, double puckRadius) {
    // Prepare to generate random x and y positions.
    int randXDomain = world->width() - 2 * (int) robotRadius;
    int randYDomain = world->height() - 2 * (int) robotRadius;
    std::uniform_int_distribution<int> robotXrng(0, randXDomain);
    std::uniform_int_distribution<int> robotYrng(0, randYDomain);

    for (auto & entity : world->getEntities("robot")) {
        auto & transform = entity.getComponent<CTransform>();

        transform.p.x = robotRadius + robotXrng(generator);
        transform.p.y = robotRadius + robotYrng(generator);
    }

    randXDomain = world->width() - 2*(int)puckRadius;
    randYDomain = world->height() - 2*(int)puckRadius;
    std::uniform_int_distribution<int> puckXrng(0, randXDomain);
    std::uniform_int_distribution<int> puckYrng(0, randYDomain);

    for (auto & entity : world->getEntities("red_puck")) { 
        auto & transform = entity.getComponent<CTransform>();

        transform.p.x = puckRadius + puckXrng(generator);
        transform.p.y = puckRadius + puckYrng(generator);
    }
}

class MyExperiment
{
    ExperimentConfig            m_config;
    ControllerConfig            m_ctrlConfig;
    int                         m_trialIndex;

    std::shared_ptr<GUI>        m_gui;
    std::shared_ptr<Simulator>  m_sim;
//    std::shared_ptr<World>      m_world;

    size_t                      m_simulationSteps = 0;
    double                      m_simulationTime = 0;
    Timer                       m_simTimer;
    std::ofstream               m_fout;

    std::stringstream           m_status;
    double                      m_eval;

    std::default_random_engine  m_rng;
    bool                        m_aborted;

    void addRobotControllers()
    {
        for (auto e : world->getEntities("robot")) {
            /*
            ControllerConfig controllerConfig;
            controllerConfig.leftRobotHiVariant = m_leftRobotHiVariant;
            controllerConfig.rightRobotHiVariant = m_rightRobotHiVariant;
            controllerConfig.leftRobotLoVariant = m_leftRobotLoVariant;
            controllerConfig.rightRobotLoVariant = m_rightRobotLoVariant;

            controllerConfig.puckVariant = m_puckVariant;
            controllerConfig.thresholdVariant = m_thresholdVariant;
            controllerConfig.defaultVariant = m_defaultVariant;
            */
            e.addComponent<CController>(std::make_shared<EntityController_OrbitalConstructionVariant>(e, world, m_config.threshold, 
                                                                                                      m_ctrlConfig));
        }

        if (m_gui) {
            m_gui->addContour(m_config.threshold);
            m_gui->addContour(m_config.threshold/2);
        }
    }

    void resetSimulator()
    {
        m_aborted = false;

        // We run out of memory if we don't do this.
        // EntityMemoryPool::Reset();

        if (world == NULL) {
            if (m_config.worldName == "lgtv") {
                world = orbital_av_world::GetLGTVWorld(m_config);
            } else {
                std::cerr << "Problem: unknown world parameter in config file\n";            
            }
        } else {
            // Entities in world need to be randomly repositioned.
            randomizeWorld(m_rng, m_config.robotRadius, m_config.puckRadius);
        }

        m_sim = std::make_shared<Simulator>(world);

        if (m_gui) {
            m_gui->setSim(m_sim);
        }
        else if (m_config.gui) {
            m_gui = std::make_shared<GUI>(m_sim, 144);
            m_gui->setDownArrowCallback(decreaseRenderSpeed);
            m_gui->setUpArrowCallback(increaseRenderSpeed);
            m_gui->setSpaceCallback(togglePause);
        }

        addRobotControllers();
    }

    void writeToFile() {
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
    
public:

    MyExperiment(ExperimentConfig config, ControllerConfig ctrlConfig, int trialIndex, int rngSeed)
        : m_config(config)
        , m_ctrlConfig(ctrlConfig)
        , m_trialIndex(trialIndex)
        , m_rng(rngSeed)
        , m_aborted(false)
    {
        if (m_config.writePlotSkip)
        {
            std::stringstream plotFilename;
            plotFilename << m_config.plotFilenameBase << "_" << trialIndex << ".dat";
            m_fout = std::ofstream(plotFilename.str());
        }

        resetSimulator();
    }

    void doSimulationStep()
    {
        if (m_config.writePlotSkip && m_simulationSteps % m_config.writePlotSkip == 0)
        {
            writeToFile();
        }
        
        ++m_simulationSteps;

        if (!m_gui && (m_simulationSteps % 20000 == 0))
        {
            std::cout << "Simulation Step: " << m_simulationSteps << "\n";
        }

        // control robots that have controllers
        for (auto & robot : m_sim->getWorld()->getEntities("robot"))
        {
            if (!robot.hasComponent<CController>()) { continue; }

            // get the action that should be done for this entity
            EntityAction action = robot.getComponent<CController>().controller->getAction();

            // have the action apply its effects to the entity
            action.doAction(robot, simTimeStepAdjusted);
        }

        // call the world physics simulation update
        // parameter = how much sim time should pass (default 1.0)
        m_sim->update(simTimeStepAdjusted);
    }

    void run()
    {
        bool running = true;
        while (running)
        {
            // Figure out what evaluation to use depending upon the type of world we're running.
            if (m_config.worldName == "lgtv") {
                m_eval = MyEval::PuckGridValues(world, "red_puck", 1, true);
            } /*else if (m_config.worldName == "simple" || m_config.worldName == "wall") {
                m_eval = MyEval::PuckSSDFromIdealPosition(world, "red_puck", Vec2(300,300));
            } else if (m_config.worldName == "symmetric") {
                m_eval = MyEval::PuckSSDFromIdealPosition(world, "red_puck", Vec2(150,150))
                       + MyEval::PuckSSDFromIdealPosition(world, "green_puck", Vec2(450,150));
            } */else {
                std::cerr << "Problem: unknown world parameter in config file\n";            
            }
            if (isnan(m_eval)) {
                std::cerr << "nan evaluation encountered!  Re-starting run with new random seed.\n";
                m_aborted = true;
                break;
            }
            
            m_simTimer.start();
            for (size_t i = 0; i < renderStepsAdjusted; i++)
            {
                if (m_config.maxTimeSteps > 0 && m_simulationSteps >= m_config.maxTimeSteps)
                {
                    running = false;
                }

                doSimulationStep();
            }
            m_simulationTime += m_simTimer.getElapsedTimeInMilliSec();

            if (m_gui)
            {
                // update gui status text
                m_status = std::stringstream();
                m_status << "Sim Steps:  " << m_simulationSteps << "\n";
                //m_status << "Sim / Sec:  " << m_simulationSteps * 1000 / m_simulationTime << "\n";
                m_status << "Puck Eval:  " << m_eval << "\n";
                m_gui->setStatus(m_status.str());

                // draw gui
                m_gui->update();
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
        std::stringstream ss;
        ss << "gnuplot/results_form_" << m_config.numRobots << "_" << m_config.maxTimeSteps << ".txt";
        std::cout << "Printing Results to: " << ss.str() << "\n";

        std::ofstream fout(ss.str());
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

};


namespace MyExperiments
{
    double runExperiment(ExperimentConfig config, ControllerConfig ctrlConfig) {
        // BAD: These are declared at the top of this file and therefore don't belong to this class specifically.
        // They are here just as a way of getting through the weirdness of defining the GUI callbacks.
        renderStepsAdjusted = config.renderSteps;
        simTimeStepAdjusted = config.simTimeStep;
 
        double avgEval = 0;
        for (int i=0; i<config.numTrials; i++) {
            //std::cout << "Trial: " << i << "\n";

            // A trial may be aborted due to receiving a Not-A-Number (nan) evaluation.  If so
            // rerun with an adjusted random seed.  Stepping by 1000 because we're not likely to
            // run 1000 trials.
            bool aborted = true;
            for (int seedAdjustment=0; aborted; seedAdjustment += 1000) {
                MyExperiment exp(config, ctrlConfig, i, i+seedAdjustment);
                exp.run();
                exp.printResults();
                aborted = exp.wasAborted();
                if (!aborted) {
                    //cerr << exp.getEvaluation() << endl;
                    avgEval += exp.getEvaluation();
                }

                // Give up on this experiment (conduct no more trials) if the last evaluation was poor.
                //if (exp.getEvaluation() > 5000) {
                //    avgEval = 1000000;
                //    i = config.numTrials; // Causing us to exit the loop
                //}
            }

        }

        ctrlConfig.print();
        std::cout << "\t" << avgEval/config.numTrials << "\n";

        return avgEval / config.numTrials;         
    }

    double runWithDefaultConfig(ControllerConfig ctrlConfig) {
        // Read the config file name from console if it exists
        std::string configFile = "lgtv_config.txt";
        ExperimentConfig config;
        config.load(configFile);

        return runExperiment(config, ctrlConfig);
    }
};