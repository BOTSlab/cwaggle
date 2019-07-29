#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CWaggle.h"
#include "worlds.hpp"
#include "controllers.hpp"
#include "MyEval.hpp"
#include "MyExperiment.hpp"

// Pagmo-specific

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/exceptions.hpp>
#include <pagmo/io.hpp>
#include <pagmo/rng.hpp>
#include <pagmo/types.hpp>
using namespace pagmo;

class MyProblem
{
public:
    MyProblem() {}

     vector_double fitness(const vector_double &x) const {
      for (int i=0; i<3; i++)
        assert(x[i] >= 0 && x[i] <= 63);
      
      return {MyExperiments::runWithDefaultConfig((int) x[0], (int) x[1], (int) x[2])};
//cout << x[0] << "\t" << x[1] << "\t" << x[2] << endl;
//return {0.25};
    }

    std::pair<vector_double, vector_double> get_bounds() const {
        vector_double lb(3, 0);
        vector_double ub(3, 63);
        return {lb, ub};
    }

    vector_double::size_type get_nx() const { return 3; }
    vector_double::size_type get_nix() const { return 3; }
    vector_double::size_type get_ncx() const { return 0; }
};

void runGA() {
    // 1 - Instantiate a pagmo problem constructing it from a UDP
    // (user defined problem).
    MyProblem prob;

    // 2 - Instantiate a pagmo algorithm
    algorithm algo{sade(100)};

    // 3 - Instantiate an archipelago with 6 islands having each 20 individuals
    archipelago archi{1, algo, prob, 200};

    // 4 - Run the evolution in parallel on the 6 separate islands 1 times.
    archi.evolve(1);

    // 5 - Wait for the evolutions to be finished
    archi.wait_check();

    // 6 - Print the fitness of the best solution in each island
    for (const auto &isl : archi) {
        std::cout << "\nBEST:\n" 
                  << isl.get_population().champion_x()[0] << '_'
                  << isl.get_population().champion_x()[1] << '_'
                  << isl.get_population().champion_x()[2] << '\t'
                  << isl.get_population().champion_f()[0] << '\n';
    }
}

int main(int argc, char ** argv)
{   
   if (argc != 1 && argc != 5) {      
      cerr << "Usage\n\t[GA MODE] cwaggle_orbital_av" << endl;
      cerr << "OR\n\t[MANUAL MODE] cwaggle_orbital_av CONFIG_FILE PARAMETERS_FOR_RUN (3)" << endl;
      return -1;
   }

   if (argc == 1) {
      runGA();
   } else if (argc == 5) {
      // Manual run with parameters specified.
      std::string configFile = argv[1];
      MyExperimentConfig config;
      config.load(configFile);

      int puckVariant = atoi(argv[2]);
      int thresholdVariant = atoi(argv[3]);
      int defaultVariant = atoi(argv[4]);
      cerr << "puckVariant: " << puckVariant << endl;
      cerr << "thresholdVariant: " << thresholdVariant << endl;
      cerr << "defaultVariant: " << defaultVariant << endl;
      MyExperiments::runExperiment(config, puckVariant, thresholdVariant, defaultVariant);
   }

   return 0;
}
