#!/usr/bin/env python
import os
#import sys
import string
import datetime
#from socket import gethostname
#from tempfile import mkdtemp

# For coloured text output to console
#from colorama import init, Fore, Style
#init()

def execute(cmd):
    print(cmd)
    os.system(cmd)

def get_time_string():
    timeString = str(datetime.datetime.now())
    timeString = string.rsplit(timeString, '.', 1)[0]
    return timeString

def main():
    cmd = "echo STARTED {} > /tmp/lastGA.dat".format(get_time_string())
    execute(cmd)
    cmd = "./cwaggle_orbital_av >> /tmp/lastGA.dat"
    execute(cmd)
    cmd = "echo STOPPED {} >> /tmp/lastGA.dat".format(get_time_string())

main()
