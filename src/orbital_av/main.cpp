#include "CWaggle.h"
#include "custom_world.hpp"
#include "controllers.hpp"
#include "MyEval.hpp"
#include "MyExperiment.hpp"
#include <math.h>

/*
void OrbitalConstructionExample(int argc, char ** argv)
{
    auto world = orbital_av_world::GetSymmetricWorld(20,  // number of robots
                                                     10,  // robot radius
                                                     10,  // sensor radius
                                                     100, // number of pucks
                                                     12); // puck radius

    std::default_random_engine rng; // Random number generator

    // add orbital controllers to all the robots
    for (auto e : world->getEntities("robot"))
    {
        e.addComponent<CController>(std::make_shared<EntityController_OrbitalConstruction2>(e, world, rng));
    }

    // create a new simulator with the given world
    auto simulator = std::make_shared<Simulator>(world);

    // set up a GUI to visualize the simulation with a given frame rate limit
    // the frame rate limit should be set at least as high as your monitor refresh rate
    // this is completely optional, simulation will run with no visualization
    // GUI can also be created at any time to start visualizing an ongoing simulation
    GUI gui(simulator, 144);

    // determines the amount of 'time' that passes per simulation tick
    // lower is more 'accurate', but 'slower' to get to a given time
    double simulationTimeStep = 1.0;

    // how many simulation ticks are peformed before each world render in the GUI
    double stepsPerRender = 5;

    // read that value from console if it exists
    if (argc == 2)
    {
        std::stringstream ss(argv[1]);
        ss >> stepsPerRender;
        if (stepsPerRender < 1.0) {
            simulationTimeStep = stepsPerRender;
            stepsPerRender = 1;
        }
    }

    // run the simulation and gui update() function in a loop
    while (true)
    {
        for (size_t i = 0; i < stepsPerRender; i++)
        {
            for (auto & robot : simulator->getWorld()->getEntities("robot"))
            {
                if (!robot.hasComponent<CController>()) { continue; }

                // get the action that should be done for this entity
                EntityAction action = robot.getComponent<CController>().controller->getAction();

                // have the action apply its effects to the entity
                action.doAction(robot, simulationTimeStep);
            }

            // call the world physics simulation update
            // parameter = how much sim time should pass (default 1.0)
            simulator->update(simulationTimeStep);
        }

        // if a gui exists, call for its display to update
        // note: simulation is limited by gui frame rate limit
        gui.update();

        //double ssd = MyEval::PuckSSDFromIdealPosition(world, "red_puck", Vec2(300,300));
        //std::cout << ssd << "\n";
    }
}
*/

int main(int argc, char ** argv)
{
    // Robots guided by orbital construction algorithm

//    OrbitalConstructionExample(argc, argv);

    MyExperiments::MainExperiment(argc, argv);

    return 0;
}
