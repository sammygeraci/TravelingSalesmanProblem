# TSP graph generator, written by Ralph W. Eide III
# This program takes in two arguments: The size of the graph (n), and the output file name.

import sys
import random

argc = len(sys.argv)

if (argc < 3):
    print("Usage: TSP_graphgen.py graph_size output_path.graph")
else:
    n = int(sys.argv[1])
    output_path = sys.argv[2]
    with open(output_path, "w") as file:
        for i in range(n):
            line = ""
            for j in range(i):
                line += str(int(((random.random()) * 499) + 1)) + " "
            line += "0\n"
            file.write(line)