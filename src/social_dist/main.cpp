#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CWaggle.h"

#include "SortExperiment.hpp"

int main(int argc, char** argv)
{
    if (argc != 1) {
        std::cerr << "Usage\n\tcwaggle_social_dist" << std::endl;
        return -1;
    }

    ControllerConfig ctrlConfig;
    SortExperiments::runWithDefaultConfig(ctrlConfig);

    return 0;
}