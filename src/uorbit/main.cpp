#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CWaggle.h"
#include "MyExperiment.hpp"

using namespace std;

void printResults(const vector<tuple<double, double, double>>& sweepResults) 
{
    for (const tuple<double, double, double> result : sweepResults) {
        cout << "K1, K2: " << get<0>(result) << ", " << get<1>(result);
        cout << " result: " << get<2>(result) << endl;
    }
}

double runExperiment(ExperimentConfig config, ControllerConfig ctrlConfig)
{
    double avgEval = 0;
    for (int i = 0; i < config.nTrials; i++) {
        cerr << "Trial: " << i << "\n";
        MyExperiment exp(config, ctrlConfig, i, i);
        exp.run();
        if (exp.wasAborted())
            cerr << "Trial aborted." << "\n";
        else
            avgEval += exp.getEvaluation();
    }

    ctrlConfig.print();
    cout << "\t" << avgEval / config.nTrials << "\n";

    return avgEval / config.nTrials;
}


double runWithDefaultConfig()
{
    ControllerConfig ctrlConfig;

    // Read the config file name from console if it exists
    string configFile = "uorbit_config.txt";
    ExperimentConfig config;
    config.load(configFile);

    return runExperiment(config, ctrlConfig);
}

void paramSweep()
{
    string configFile = "uorbit_config.txt";
    ExperimentConfig config;
    config.load(configFile);

    // Leaving it as default.
    ControllerConfig ctrlConfig;

    vector<pair<int, double>> sweepResults;
    for (config.nRobots = 1; config.nRobots < 15; config.nRobots += 5) {
        ostringstream oss;
        oss << "../../data/" << config.nRobots;
        config.plotFilenameBase = oss.str();

        cout << "nRobots: " << config.nRobots << endl;
        double avgEval = runExperiment(config, ctrlConfig);
        sweepResults.push_back(make_tuple(config.nRobots, avgEval));
    }

    for (const pair<int, double> result : sweepResults)
        cout << "nRobots: " << result.first << ": " << result.second << endl;

/*
    vector<tuple<double, double, double>> sweepResults;
    ControllerConfig ctrlConfig;
    for (ctrlConfig.K1 = 0.1; ctrlConfig.K1 < 1; ctrlConfig.K1 += 0.1) {
        cout << "K1: " << ctrlConfig.K1 << endl;
        for (ctrlConfig.K2 = 0.1; ctrlConfig.K2 < 1; ctrlConfig.K2 += 0.1) {
            cout << "K2: " << ctrlConfig.K2 << endl;
            double avgEval = runExperiment(config, ctrlConfig, false);
            sweepResults.push_back(make_tuple(ctrlConfig.K1, ctrlConfig.K2, avgEval));
        }
    }

    cout << "\n\nALL RESULTS: \n" << endl;
    printResults(sweepResults);

    cout << "\n\nSORTED RESULTS: \n" << endl;
    sort(sweepResults.begin(), sweepResults.end(),
        [](const tuple<double, double, double>& lhs, const tuple<double, double, double>& rhs) {
            return get<2>(lhs) < get<2>(rhs);
        }
    );
    printResults(sweepResults);
*/
}


int main(int argc, char** argv)
{
    if (argc != 2) {
        cerr << "Usage\n\tcwaggle_uorbit [run or sweep]" << endl;
        return -1;
    }
    string cmd(argv[1]);

    if (cmd == "run")
        runWithDefaultConfig();
    else if (cmd == "sweep")
        paramSweep();
    else {
        cerr << "Unknown command: " << cmd << endl;
        return -1;
    }

    return 0;
}