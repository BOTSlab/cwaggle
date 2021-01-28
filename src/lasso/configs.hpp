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
    double plowAngleDeg   = 0.0;

    double c1Radius     = 80.0;
    double c1Distance   = 140.0;
    double c2Radius     = 10000.0;
    double c2Distance   = 10000.0;

    double robotViewRadius     = 1000.0;

    size_t numPucks     = 0;
    double puckRadius   = 10.0;

    size_t wallArrangement = 0;

    double threshold    = 0.1;

    // Simulation Parameters
    double simTimeStep  = 1.0;
    double renderSteps  = 1;
    size_t maxTimeSteps = 0;

    size_t writePlotSkip    = 0;
    std::string plotFilenameBase   = "";
    size_t numTrials = 10;
    std::string evalName = "";
    std::string grid0Filename = "";
    std::string grid1Filename = "";
    std::string grid2Filename = "";

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
            else if (token == "plowAngleDeg")    { fin >> plowAngleDeg; }
            else if (token == "c1Radius")    { fin >> c1Radius; }
            else if (token == "c1Distance")    { fin >> c1Distance; }
            else if (token == "c2Radius")    { fin >> c2Radius; }
            else if (token == "c2Distance")    { fin >> c2Distance; }
            else if (token == "robotViewRadius")    { fin >> robotViewRadius; }
            else if (token == "gui")            { fin >> gui; }
            else if (token == "numPucks")       { fin >> numPucks; }
            else if (token == "puckRadius")     { fin >> puckRadius; }
            else if (token == "wallArrangement")       { fin >> wallArrangement; }
            else if (token == "simTimeStep")    { fin >> simTimeStep; }
            else if (token == "renderSkip")     { fin >> renderSteps; }
            else if (token == "maxTimeSteps")   { fin >> maxTimeSteps; }
            else if (token == "writePlotSkip")  { fin >> writePlotSkip; }
            else if (token == "plotFilenameBase")   { fin >> plotFilenameBase; }
            else if (token == "numTrials")   { fin >> numTrials; }
            else if (token == "evalName")   { fin >> evalName; }
            else if (token == "grid0Filename")   { fin >> grid0Filename; }
            else if (token == "grid1Filename")   { fin >> grid1Filename; }
            else if (token == "grid2Filename")   { fin >> grid2Filename; }
            else if (token == "fakeRobots")   { fin >> fakeRobots; }
            else if (token == "captureScreenshots")   { fin >> captureScreenshots; }
            else if (token == "screenshotFilenameBase")   { fin >> screenshotFilenameBase; }
        }
    }
};

struct ControllerConfig
{
    double maxForwardSpeed = 2;
    double maxAngularSpeed = 0.5; // 2.0;
    double Komega = 1.0; // 0.05;
    double Kbal = 0.15;
    double maxSensingDistance = 250;
    double fieldOfView = M_PI;

    ControllerConfig() {}

    void load(const std::string & filename)
    {
        std::ifstream fin(filename);
        std::string token;
        double tempVal = 0;
 
        while (fin.good())
        {
            fin >> token;
            if (token == "maxForwardSpeed")    { fin >> maxForwardSpeed; }
            else if (token == "maxAngularSpeed")    { fin >> maxAngularSpeed; }
            else if (token == "Komega")    { fin >> Komega; }
            else if (token == "Kbal")    { fin >> Kbal; }
            else if (token == "maxSensingDistance")    { fin >> maxSensingDistance; }
            else if (token == "fieldOfView")    { fin >> fieldOfView; }
        }
    }

    void print() {
        std::cout 
            << maxForwardSpeed << "_" << maxAngularSpeed << "_" << Komega << "_" << Kbal;
    }
};