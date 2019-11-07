#pragma once

#include <string>
#include <fstream>
#include <iostream>

struct ExperimentConfig
{
    size_t gui          = 1;
    size_t numRobots    = 20;
    double robotRadius  = 10.0;

    double plowLength   = 60.0;
    double c1Radius     = 80.0;
    double c1Distance   = 140.0;

    double puckViewRadius     = 1000.0;
    double robotViewRadius     = 1000.0;

    size_t numPucks     = 0;
    double puckRadius   = 10.0;

    double threshold    = 0.1;

    // Simulation Parameters
    double simTimeStep  = 1.0;
    double renderSteps  = 1;
    size_t maxTimeSteps = 0;

    size_t writePlotSkip    = 0;
    std::string plotFilenameBase   = "";
    size_t numTrials = 10;
    std::string worldName = "";
    std::string gridFilename = "";
    std::string evalGridFilename = "";

    size_t fakeRobots          = 0;
    size_t captureScreenshots          = 0;
    std::string screenshotFilenameBase   = "";

    ExperimentConfig() {}

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
            else if (token == "puckViewRadius")    { fin >> puckViewRadius; }
            else if (token == "robotViewRadius")    { fin >> robotViewRadius; }
            else if (token == "gui")            { fin >> gui; }
            else if (token == "numPucks")       { fin >> numPucks; }
            else if (token == "puckRadius")     { fin >> puckRadius; }
            else if (token == "threshold")    { fin >> threshold; }
            else if (token == "simTimeStep")    { fin >> simTimeStep; }
            else if (token == "renderSkip")     { fin >> renderSteps; }
            else if (token == "maxTimeSteps")   { fin >> maxTimeSteps; }
            else if (token == "writePlotSkip")  { fin >> writePlotSkip; }
            else if (token == "plotFilenameBase")   { fin >> plotFilenameBase; }
            else if (token == "numTrials")   { fin >> numTrials; }
            else if (token == "worldName")   { fin >> worldName; }
            else if (token == "gridFilename")   { fin >> gridFilename; }
            else if (token == "evalGridFilename")   { fin >> evalGridFilename; }
            else if (token == "fakeRobots")   { fin >> fakeRobots; }
            else if (token == "captureScreenshots")   { fin >> captureScreenshots; }
            else if (token == "screenshotFilenameBase")   { fin >> screenshotFilenameBase; }
        }
    }
};

struct ControllerConfig
{
    int puckVariant;
    int alignVariant;
    bool stallDetection;

    ControllerConfig() {}

    void load(const std::string & filename)
    {
        std::ifstream fin(filename);
        std::string token;
        double tempVal = 0;
 
        while (fin.good())
        {
            fin >> token;
            if (token == "puckVariant")    { fin >> puckVariant; }
            else if (token == "alignVariant")    { fin >> alignVariant; }
            else if (token == "stallDetection")    { fin >> stallDetection; }
        }
    }

    void print() {
        std::cout 
            << puckVariant << "_" << alignVariant << "_" << stallDetection;
    }
};