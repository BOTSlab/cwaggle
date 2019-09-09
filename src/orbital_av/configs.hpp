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
    double c2Radius     = 40.0;
    double c2Distance   = 140.0; 
    double c3Radius     = 1000.0;

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
            else if (token == "c2Radius")    { fin >> c2Radius; }
            else if (token == "c2Distance")    { fin >> c2Distance; }
            else if (token == "c3Radius")    { fin >> c3Radius; }
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
        }
    }
};

struct ControllerConfig
{
    int leftRobotHiVariant;
    int rightRobotHiVariant;
    int leftRobotLoVariant;
    int rightRobotLoVariant;

    int puckVariant;
    int thresholdVariant;
    int defaultVariant;

    ControllerConfig() {}

    void load(const std::string & filename)
    {
        std::ifstream fin(filename);
        std::string token;
        double tempVal = 0;
 
        while (fin.good())
        {
            fin >> token;
            if (token == "leftRobotHiVariant")      { fin >> leftRobotHiVariant; }
            else if (token == "rightRobotHiVariant")    { fin >> rightRobotHiVariant; }
            else if (token == "leftRobotLoVariant")    { fin >> leftRobotLoVariant; }
            else if (token == "rightRobotLoVariant")    { fin >> rightRobotLoVariant; }
            else if (token == "puckVariant")    { fin >> puckVariant; }
            else if (token == "thresholdVariant")    { fin >> thresholdVariant; }
            else if (token == "defaultVariant")    { fin >> defaultVariant; }
        }
    }

    void print() {
        std::cout 
            << leftRobotHiVariant << "_" << rightRobotHiVariant << "_" << leftRobotLoVariant << "_" << rightRobotLoVariant << "___"
            << puckVariant << "_" << thresholdVariant << "_" << defaultVariant;
    }
};