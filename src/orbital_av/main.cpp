//=================================================================================================
//                    Copyright (C) 2017 Olivier Mallet - All Rights Reserved                      
//=================================================================================================

#include "CWaggle.h"
#include "worlds.hpp"
#include "controllers.hpp"
#include "MyEval.hpp"
#include "MyExperiment.hpp"
#include "GALGO-2.0/Galgo.hpp"

//using namespace MyExperiments;

// objective class example
template <typename T>
class MyObjective
{
public:
   // objective function example : Rosenbrock function
   // minimizing f(x,y) = (1 - x)^2 + 100 * (y - x^2)^2
   static std::vector<T> Objective(const std::vector<T>& x)
   {
      //T obj = -(pow(1-x[0],2)+100*pow(x[1]-x[0]*x[0],2));

      T obj = MyExperiments::runWithDefaultConfig((int) x[0], (int) x[1], (int) x[2]);
      return {obj};
   }
   // NB: GALGO maximize by default so we will maximize -f(x,y)
};

// constraints example:
// 1) x * y + x - y + 1.5 <= 0
// 2) 10 - x * y <= 0
/*
template <typename T>
std::vector<T> MyConstraint(const std::vector<T>& x)
{
   return {x[0]*x[1]+x[0]-x[1]+1.5,10-x[0]*x[1]};
}
*/
// NB: a penalty will be applied if one of the constraints is > 0 
// using the default adaptation to constraint(s) method


void runGA() {
   // initializing parameters lower and upper bounds
   // an initial value can be added inside the initializer list after the upper bound
   galgo::Parameter<double, 6> par1({1,64});
   galgo::Parameter<double, 6> par2({1,64});
   galgo::Parameter<double, 6> par3({1,64});
   // here both parameter will be encoded using 16 bits the default value inside the template declaration
   // this value can be modified but has to remain between 1 and 64

   // initiliazing genetic algorithm
   galgo::GeneticAlgorithm<double> ga(MyObjective<double>::Objective,100,500,true,par1,par2,par3);

   // setting constraints
   //ga.Constraint = MyConstraint;

   // running genetic algorithm
   ga.run();
}

int main(int argc, char ** argv)
{   
   // Read the config file name from console if it exists
   std::string configFile = "lgtv_config.txt";

   if (argc != 2 && argc != 5) {      
      cerr << "Usage\n\t[GA MODE] cwaggle_orbital_av CONFIG_FILE" << endl;
      cerr << "OR\n\t[MANUAL MODE] cwaggle_orbital_av CONFIG_FILE PARAMETERS_FOR_RUN (3)" << endl;
      return -1;
   }

   configFile = argv[1];
   MyExperimentConfig config;
   config.load(configFile);

   if (argc == 2) {
      runGA();
   } else if (argc == 5) {
      // Manual run with parameters specified.
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