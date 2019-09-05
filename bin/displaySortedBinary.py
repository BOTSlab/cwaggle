#!/usr/bin/env python
import sys

def main():
    if len(sys.argv) != 2:
        print("Usage:\n\tdisplaySortedBinary FILENAME")
        return

    filename = sys.argv[1]
    f = open(filename, "r")

    for line in f:
        tokens = line.strip().split(" ")
        variantParts = tokens[1].split("_")
        variantStr = "{}_{}_{}".format(int(variantParts[0]), int(variantParts[1]), int(variantParts[2]))
        strBin = "{0:0^6b}"
        variantStrBin = strBin.format(int(variantParts[0])) + "_" + strBin.format(int(variantParts[1])) + "_" + strBin.format(int(variantParts[2]))
        print("{}\t{}\t{}\t{}".format(tokens[0], variantStr, variantStrBin, tokens[2]))

main()
