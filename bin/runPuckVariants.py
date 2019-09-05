#!/usr/bin/env python
import os

def execute(cmd):
    print(cmd)
    os.system(cmd)

def main():
    # For a fresh run
    first = True
    startI = 0

    # For continuing an interrupted run
    # first = False
    # startI = 3

    for i in range(startI,64):
        for j in range(0,64):
            if first:
                cmd = "./cwaggle_orbital_av simple_config.txt {} {} > results.dat".format(i, j)
                first = False
            else:
                cmd = "./cwaggle_orbital_av simple_config.txt {} {} >> results.dat".format(i, j)
            execute(cmd)

main()
