# An Enhanced Branch and Price Algorithm for the Time-Dependent Vehicle Routing Problem with Time Windows
Source code to replicate the experiments.

## Abstract
In this paper we implement a branch and price (BP) algorithm for a time dependent vehicle routing problem with time windows in which the goal is to minimize the total route duration (DM-TDVRPTW).  The travel time between two customers depends on the departure time and, thus, it need not remain fixed along the planning horizon.  We propose several improvements to the exact labeling algorithm by Dabia et al. (Branch and price for the time-dependent vehicle routing problem with time windows, _Transp. Sci._ 2013; 47(3):380--396) for solving the pricing problem, while we provide a tailored implementation for the dominance tests that relies on efficient data structures for storing the enumerated labels.  Computational results show that the proposed techniques are effective for accelerating the column generation step.  The obtained BP algorithm is able to solve benchmark instances with up to 100 customers, being able to solve all the instances with 25 customers.  Furthermore, heuristic adaptations are able to find good quality solutions in reasonable computing times, showing its potential to be applied in practice.

## Getting started
The following instructions will guide you through the steps to execute the experiments from the article.

### Prerequisites
- Python >= 3.6 [(more info)](https://www.python.org/)
- CPLEX >= 12.8 [(more info)](https://www.ibm.com/products/ilog-cplex-optimization-studio)
- Boost Graph Library >=1.66 [(more info)](https://www.boost.org/doc/libs/1_66_0/libs/graph/doc/index.html)
    - On Linux: ```sudo apt-get install libboost-all-dev```
- CMake >= 2.8.4 [(more info)](https://cmake.org/)
    - On Linux: ```sudo apt-get install cmake```
- C++14 or higher [(more info)](https://es.wikipedia.org/wiki/C%2B%2B14)

### Built with
- Kaleidoscope: A tool to visualize the outputs of Optimization Problems [(more info)](https://github.com/gleraromero/kaleidoscope)
- Runner: A script to ease the process of running experiments [(more info)](https://github.com/gleraromero/runner)
- GOC lib: A library that includes interfaces for using (Mixed Integer) Linear Programming solvers, and some useful resources [(more info)](https://github.com/gleraromero/goc).

### Running the experiments.
1. Add environment variables with the paths to the libraries.
    1. Add two environment variables to bash with CPLEX include and library paths.
        1. ```export CPLEX_INCLUDE=<path_to_cplex_include_dir>```
            - Usually on Linux: _/opt/ibm/ILOG/CPLEX_Studio\<VERSION\>/cplex/include_
        1. ```export CPLEX_BIN=<path_to_cplex_lib_binary_file>```
            - Usually on Linux: _/opt/ibm/ILOG/CPLEX_Studio\<VERSION\>/cplex/lib/x86-64_linux/static_pic/libcplex.a_
    1. Add two environment variables to bash with BOOST Graph Library include and library paths.
        1. ```export BOOST_INCLUDE=<path_to_boost_include_dir>```
            - Usually on Linux: _/usr/include_
        1. ```export BOOST_BIN=<path_to_boost_lib_binary_file>```
            - Usually on Linux: _/usr/lib/x86_64-linux-gnu/libboost_graph.a_
1. Go to the networks2019 root directory.
1. Execute ```python3 runner/runner.py <experiment_file>```
1. The execution output will be continually saved to the output folder.

> Experiment files are located in the _experiments_ folder. For more information see Section [Experiments](#Experiments)

### Experiments
There are eight experiment files. Which together comprise all the experiments carried out in the article.
* _Section 7.1_: pricing.json
* _Section 7.2_: bp.json
* _Section 7.4_: bp_heur.json

### Visualizing the experiment results.
1. Go to the folder kaleidoscope and open the file index.html with a Web Browser (Chrome prefered).
1. Add the output file.
1. Select the experiments.
1. Add some attributes to visualize.
1. Click on Refresh.
1. If more details on an experiment are desired click on the + icon in a specific row.

### Checker
We include a checker program to validate that algorithms produce **valid** routes. To run the checker execute:
```python3 checker/checker.py output/<output_file.json>```

The checker will go through each instance and validate:
- That the exact solution route is feasible (with respect to all resources).
- That the reported duration of the route is correct.
- If Optimum status is reported, then it should be better or equal than any solution in the _solutions.json_ file of its dataset.

## Built With
* [JSON for Modern C++](https://github.com/nlohmann/json)
* [Boost Graph Library](https://www.boost.org/doc/libs/1_66_0/libs/graph/doc/index.html)

## Authors
- Gonzalo Lera-Romero
- Juan José Miranda-Bront
- Francisco Soulignac

## License
This project is licensed under the MIT License - see the LICENSE.md file for details
