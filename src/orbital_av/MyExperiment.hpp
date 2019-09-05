#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <functional>

#include "CWaggle.h"
#include "GUI.hpp"
#include "MyEval.hpp"

//using namespace std;


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

struct MyExperimentConfig
{
    // Square World Parameters
    //size_t width        = 800;
    //size_t height       = 800;
    size_t gui          = 1;
    size_t numRobots    = 20;
    double robotRadius  = 10.0;

    double plowLength   = 60.0;
    double c1Radius     = 80.0;
    double c1Distance   = 140.0;
    double c2Radius     = 40.0;
    double c2Distance   = 140.0; 
    double c3Radius     = 1000.0;
    double c4Radius     = 100.0;

    size_t numPucks     = 0;
    double puckRadius   = 10.0;

    // Simulation Parameters
    double simTimeStep  = 1.0;
    double renderSteps  = 1;
    size_t maxTimeSteps = 0;

    size_t writePlotSkip    = 0;
    std::string plotFilenameBase   = "";
    size_t numTrials = 10;
    std::string worldName = "";
    std::string gridFilename = "";

    size_t controllerConfig   = 0;


    // Orbital Construction Config
    // OrbitalConstructionConfig occ;

    MyExperimentConfig() {}

    void load(const std::string & filename)
    {
        std::ifstream fin(filename);
        std::string token;
        double tempVal = 0;
 
        while (fin.good())
        {
            fin >> token;
            if (token == "numRobots")      { fin >> numRobots; }
            else if (token == "robotRadius")    { fin >> robotRadius; }
            else if (token == "plowLength")    { fin >> plowLength; }
            else if (token == "c1Radius")    { fin >> c1Radius; }
            else if (token == "c1Distance")    { fin >> c1Distance; }
            else if (token == "c2Radius")    { fin >> c2Radius; }
            else if (token == "c2Distance")    { fin >> c2Distance; }
            else if (token == "c3Radius")    { fin >> c3Radius; }
            else if (token == "c4Radius")    { fin >> c4Radius; }
            else if (token == "gui")            { fin >> gui; }
            else if (token == "numPucks")       { fin >> numPucks; }
            else if (token == "puckRadius")     { fin >> puckRadius; }
            else if (token == "simTimeStep")    { fin >> simTimeStep; }
            else if (token == "renderSkip")     { fin >> renderSteps; }
            else if (token == "maxTimeSteps")   { fin >> maxTimeSteps; }
            else if (token == "writePlotSkip")  { fin >> writePlotSkip; }
            else if (token == "plotFilenameBase")   { fin >> plotFilenameBase; }
            else if (token == "numTrials")   { fin >> numTrials; }
            else if (token == "worldName")   { fin >> worldName; }
            else if (token == "gridFilename")   { fin >> gridFilename; }
            else if (token == "controllerConfig")   { fin >> controllerConfig; }
        }

        // BAD: These don't really belong here.
        renderStepsAdjusted = renderSteps;
        simTimeStepAdjusted = simTimeStep;

    }

    std::map<std::string,double> getMap()
    {
        std::map<std::string,double> m;
        m["numRobots"] = numRobots;
        m["robotRadius"]  = robotRadius;
        m["plowLength"]   = plowLength;
        m["c1Radius"]     = c1Radius;
        m["c1Distance"]   = c1Distance;
        m["c2Radius"]     = c2Radius;
        m["c2Distance"]   = c2Distance; 
        m["c3Radius"]     = c3Radius;
        m["c4Radius"]     = c4Radius;
        m["numPucks"]     = numPucks;
        m["puckRadius"]   = puckRadius;
        return m;
    }
};




class MyExperiment
{
    MyExperimentConfig          m_config;
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

    int             m_puckVariant, m_thresholdVariant, m_defaultVariant;


    void addRobotControllers()
    {
//        std::uniform_real_distribution<double> uniformDist(0, 1.0);
//        std::normal_distribution<double> normalDist(0.0,0.1);

        // add orbital controllers to all the robots
        //std::cout << "Thresholds: ";
        double threshold = 0.1;
        for (auto e : world->getEntities("robot"))
        {
            // Determine robot's threshold.
//            double threshold;
//
//            threshold = 1 - 0.999*exp((uniformDist(m_rng) - 1.0) / 1.25);
//if (uniformDist(m_rng) < 0.75) {
//            threshold = uniformDist(m_rng);
//} else {
//            threshold = 0.015;
//}
//
//
//            std::cout << threshold << " ";
//            e.addComponent<CController>(std::make_shared<EntityController_OrbitalConstruction2>(e, m_world, threshold));


            e.addComponent<CController>(std::make_shared<EntityController_OrbitalConstructionVariant>(e, world, threshold, m_puckVariant, m_thresholdVariant, m_defaultVariant, m_config.controllerConfig));

        }

        if (m_gui) {
            m_gui->addContour(threshold);
        }
        //std::cout << std::endl;
    }

    void resetSimulator()
    {
        m_aborted = false;

        // We run out of memory if we don't do this.
        // EntityMemoryPool::Reset();

        if (world == NULL) {
            if (m_config.worldName == "lgtv") {
                world = orbital_av_world::GetLGTVWorld(m_config.gridFilename, m_config.getMap());
            } /*else if (m_config.worldName == "symmetric") {
                world = orbital_av_world::GetSymmetricWorld(
                    m_config.numRobots, m_config.robotRadius, m_config.senseRadius,
                    m_config.numPucks, m_config.puckRadius);
            } else if (m_config.worldName == "wall") {
                world = orbital_av_world::GetWallWorld(
                    m_config.numRobots, m_config.robotRadius, m_config.senseRadius,
                    m_config.numPucks, m_config.puckRadius);
            } */else {
                std::cerr << "Problem: unknown world parameter in config file\n";            
            }
        } else {
            // Entities in world need to be randomly repositioned.
            randomizeWorld(m_rng, m_config.robotRadius, m_config.puckRadius);
        }

        m_sim = std::make_shared<Simulator>(world);

        if (m_gui)
        {
            m_gui->setSim(m_sim);
        }
        else if (m_config.gui)
        {
            m_gui = std::make_shared<GUI>(m_sim, 144);
            m_gui->setDownArrowCallback(decreaseRenderSpeed);
            m_gui->setUpArrowCallback(increaseRenderSpeed);
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

//    MyExperiment(const MyExperimentConfig & config, int trialIndex, int rngSeed,
    MyExperiment(MyExperimentConfig config, int trialIndex, int rngSeed,
        int puckVariant, int thresholdVariant, int defaultVariant)
        : m_config(config)
        , m_trialIndex(trialIndex)
        , m_rng(rngSeed)
        , m_aborted(false)
        , m_puckVariant(puckVariant)
        , m_thresholdVariant(thresholdVariant)
        , m_defaultVariant(defaultVariant)
    {
        if (m_config.writePlotSkip)
        {
            std::stringstream plotFilename;
            plotFilename << m_config.plotFilenameBase << ".dat";
            //    << "_" << puckVariant
            //    << "_" << thresholdVariant
            //    << "_" << defaultVariant
            //    << "_" << trialIndex << ".txt";
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
                m_eval = MyEval::PuckGridValues(world, "red_puck", 0);
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
                m_status << "Sim / Sec:  " << m_simulationSteps * 1000 / m_simulationTime << "\n";
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
    double runExperiment(MyExperimentConfig config, int puckVariant, int thresholdVariant, int defaultVariant) {
        double avgEval = 0;
        for (int i=0; i<config.numTrials; i++) {
            //std::cout << "Trial: " << i << "\n";

            // A trial may be aborted due to receiving a Not-A-Number (nan) evaluation.  If so
            // rerun with an adjusted random seed.  Stepping by 1000 because we're not likely to
            // run 1000 trials.
            bool aborted = true;
            for (int seedAdjustment=0; aborted; seedAdjustment += 1000) {
                MyExperiment exp(config, i, i+seedAdjustment, puckVariant, thresholdVariant, defaultVariant);
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

        std::cout << puckVariant << "_" << thresholdVariant << "_" << defaultVariant << " "
                  << avgEval/config.numTrials << "\n";

        return avgEval / config.numTrials;         
    }

/*
    void MainExperiment(int argc, char ** argv)
    {
 
        // Testing different variants of OC
        int puckVariant = atoi(argv[2]);
        int thresholdVariant = atoi(argv[3]);
        int defaultVariant = atoi(argv[4]);
        cerr << "puckVariant: " << puckVariant << endl;
        cerr << "thresholdVariant: " << thresholdVariant << endl;
        cerr << "defaultVariant: " << defaultVariant << endl;
        //for (int puckVariant = 34; puckVariant < 64; puckVariant++)
        //    for (int thresholdVariant = 0; thresholdVariant < 64; thresholdVariant++) {
                int defaultVariant = 1;
            //for (int defaultVariant = 0; defaultVariant < 64; defaultVariant++)

for (int numPucks = 10; numPucks <=50; numPucks+= 10) {
config.numPucks = numPucks;
                runExperiment(config, puckVariant, thresholdVariant, defaultVariant);
}
        //    }

        runExperiment(config, puckVariant, thresholdVariant, defaultVariant);

        // The best for LGTV.
        runExperiment(config, 5, 1, 12);

        // Encodes the original OC algorithm.
        // runExperiment(config, 1, 1, 12);

        // Encodes the best of the first search for variants.
        // The best for 10 pucks and in the top 3 for 20
        //runExperiment(config, 47, 62, 1);

        // The best for 50 pucks
        //runExperiment(config, 45, 44, 1);
    }
*/        

    double runWithDefaultConfig(int puckVariant, int thresholdVariant, int defaultVariant)
    {
        // Read the config file name from console if it exists
        std::string configFile = "lgtv_config.txt";
        MyExperimentConfig config;
        config.load(configFile);
 
        //cerr << "puckVariant: " << puckVariant << endl;
        //cerr << "thresholdVariant: " << thresholdVariant << endl;
        //cerr << "defaultVariant: " << defaultVariant << endl;

        return runExperiment(config, puckVariant, thresholdVariant, defaultVariant);
    }
};
