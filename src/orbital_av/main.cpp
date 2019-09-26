#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CWaggle.h"

#include "MyExperiment.hpp"

// Pagmo-specific
#include <pagmo/algorithm.hpp>
//#include <pagmo/algorithms/sga.hpp>
//#include <pagmo/algorithms/ihs.hpp>
//#include <pagmo/algorithms/gaco.hpp> // GACO: Ant Colony Optimization cannot work with a solution archive bigger than the population size
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/io.hpp>
#include <pagmo/rng.hpp>
#include <pagmo/types.hpp>
using namespace pagmo;

const int N_PARAMETERS = 1;

// Make sure the value is constrained to the range [0, 63]
int constrain(double x) {
    if (x < 0) x = 0;
    if (x > 63) x = 63;
    return x;
}

class MyProblem
{
public:
    MyProblem() {}

     vector_double fitness(const vector_double &x) const {
      for (int i=0; i<N_PARAMETERS; i++)
        assert(x[i] >= 0 && x[i] <= 63);
      
      ControllerConfig ctrlConfig;
      //ctrlConfig.avoidVariant = constrain(x[0]);
      ctrlConfig.puckVariant = 5;
      ctrlConfig.alignVariant = 12;
      return {MyExperiments::runWithDefaultConfig(ctrlConfig)};
    }

    std::pair<vector_double, vector_double> get_bounds() const {
        vector_double lb(N_PARAMETERS, 0);
        vector_double ub(N_PARAMETERS, 63);
        return {lb, ub};
    }

    vector_double::size_type get_nx() const { return N_PARAMETERS; }
    vector_double::size_type get_nix() const { return N_PARAMETERS; }
    vector_double::size_type get_ncx() const { return 0; }
};

void runGA() {
    MyProblem prob;

    int nGenerations = 500;
    //algorithm algo{sga(nGenerations)};
    //algorithm algo{ihs(nGenerations)};
    //algorithm algo{gaco(nGenerations)};
    algorithm algo{sade(nGenerations)};

    population pop{prob, 24};
    pop = algo.evolve(pop);
    std::cout << "The population: \n" << pop;

    // 3 - Instantiate an archipelago 
    //archipelago archi{1, algo, prob, 200};

    // 4 - Run the evolution in parallel on the 6 separate islands 1 times.
    //archi.evolve(1);

    // 5 - Wait for the evolutions to be finished
    // archi.wait_check();


    // 6 - Print the fitness of the best solution in each island
    /*
    for (const auto &isl : archi) {
        std::cout << "\nBEST:\n" 
                  << isl.get_population().champion_x()[0] << '_'
                  << isl.get_population().champion_x()[1] << '_'
                  << isl.get_population().champion_x()[2] << '_'
                  << isl.get_population().champion_x()[3] << '\t'
                  << isl.get_population().champion_f()[0] << '\n';
    }
    */
}

void parameterSweep()
{
    ControllerConfig ctrlConfig;
    ctrlConfig.stallDetection = false;
    for (ctrlConfig.puckVariant = 0; ctrlConfig.puckVariant < 64; ctrlConfig.puckVariant++)
        for (ctrlConfig.alignVariant = 0; ctrlConfig.alignVariant < 64; ctrlConfig.alignVariant++)
            MyExperiments::runWithDefaultConfig(ctrlConfig);

    /*
    ctrlConfig.stallDetection = true;
    for (ctrlConfig.puckVariant = 0; ctrlConfig.puckVariant < 64; ctrlConfig.puckVariant++)
        for (ctrlConfig.alignVariant = 0; ctrlConfig.alignVariant < 64; ctrlConfig.alignVariant++)
            MyExperiments::runWithDefaultConfig(ctrlConfig);
    */

    /*
    ctrlConfig.puckVariant = 55;//5;
    ctrlConfig.alignVariant = 32;//12;
    ctrlConfig.stallDetection = true;
    MyExperiments::runWithDefaultConfig(ctrlConfig);
    ctrlConfig.stallDetection = false;
    MyExperiments::runWithDefaultConfig(ctrlConfig);
    */
}

void runBest()
{
    ControllerConfig ctrlConfig;
    ctrlConfig.puckVariant = 13;//5;
    ctrlConfig.alignVariant = 18;//12;
    ctrlConfig.stallDetection = false;
    MyExperiments::runWithDefaultConfig(ctrlConfig);
}

int main(int argc, char ** argv)
{   
   if (argc != 1 && argc != 6) {      
      std::cerr << "Usage\n\t[GA MODE] cwaggle_orbital_av" << std::endl;
      std::cerr << "OR\n\t[MANUAL MODE] cwaggle_orbital_av CONFIG_FILE PARAMETERS_FOR_RUN (7)" << std::endl;
      return -1;
   }

   if (argc == 1) {
      //runGA();
      //parameterSweep();
      runBest();
   } /*else if (argc == 6) {
      // Manual run with parameters specified.
      std::string configFile = argv[1];
      ExperimentConfig config;
      config.load(configFile);

      ControllerConfig ctrlConfig;
      ctrlConfig.avoidVariant = atoi(argv[2]);
      ctrlConfig.puckVariant = atoi(argv[3]);
      ctrlConfig.thresholdVariant = atoi(argv[4]);
      ctrlConfig.defaultVariant = atoi(argv[5]);
      std::cerr << "avoidVariant: " << ctrlConfig.avoidVariant << std::endl;
      std::cerr << "puckVariant: " << ctrlConfig.puckVariant << std::endl;
      std::cerr << "thresholdVariant: " << ctrlConfig.thresholdVariant << std::endl;
      std::cerr << "defaultVariant: " << ctrlConfig.defaultVariant << std::endl;
      MyExperiments::runExperiment(config, ctrlConfig);
   }
   */

   return 0;
}