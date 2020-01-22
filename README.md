# Fault Detection and Diagnosis (FDD) 
Observer-based Fault Detection and Diagnostic scheme

## Abstract
This code presents the design of a fault detection and diagnosis (FDD) scheme, composed from a bank of two types of dedicated observers, applied to linear parameter varying (LPV) systems. 
* The first one uses a combination of reduced-order LPV observers to detect, isolate and estimate actuators faults.

<center><img src="https://github.com/ebernardi/LPVRUIOBank.png" width="300"></center>

* The second one consists of a set of full-order LPV unknown input observers (UIO) to detect, isolate and estimate sensors faults. 

<center><img src="https://github.com/ebernardi/LPVUIOOBank.png" width="300"></center>

The observer's design, convergence and its stability conditions are guaranteed in terms of linear matrix inequalities (LMI). Therefore, the main purpose of this work is to provide a novelty model-based observers' technique to detect and diagnose faults upon non-linear systems.

Simulation results, based on two typical chemical industrial processes, are given to illustrate and discuss the implementation and performance of such approach.

## Requirements
- At least an i5-3337U CPU@2.7 GHz (2 Cores) with 6 GB of RAM.
- Matlab software R2016b or greater (https://mathworks.com/)

### Packages:
- Yalmip (https://yalmip.github.io/)
- Gurobi (https://www.gurobi.com/)
- SeDuMi (http://sedumi.ie.lehigh.edu/)
- LMI Lab 
