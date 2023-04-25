# K-CBS-Demos
Demonstrations of our Kinodynamic Conflict-Based Search (K-CBS) algorithm that are too involved to be included in OMPL. Please see the [References](#references) for detailed explanations of the algorithm. The implementation has been built into [The Open Motion Planning Library](https://ompl.kavrakilab.org/index.html) but the pull request has not yet been finalized. In the meantime, the library with the K-CBS implementation can be installed at [this repository](https://github.com/aria-systems-group/Kinodynamic-Conflict-Based-Search).

## Set-Up
To run these demos, one must first install [this OMPL repository](https://github.com/aria-systems-group/Kinodynamic-Conflict-Based-Search). After that, one can build this project via the following commands.
```
cd K-CBS-Demos
mkdir build/
cd build/
cmake ..
make
```
This will build many executables that can be run individually. For example, to see how quickly K-CBS can solve an MRMP problem instance with 10 2nd order cars in an empty 32x32 workspace, one can enter the following command:
```
./demo_Empty32x32_10robots_dyn2ndOrderCars 
```

## References
1. J. Kottinger, S. Almagor and M. Lahijanian, "Conflict-Based Search for Multi-Robot Motion Planning with Kinodynamic Constraints," 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Kyoto, Japan, 2022, pp. 13494-13499, doi: 10.1109/IROS47612.2022.9982018.
2. Theurkauf, Anne, Kottinger, Justin, and Lahijanian, Morteza. "Chance-Constrained Multi-Robot Motion Planning under Gaussian Uncertainties." arXiv preprint arXiv:2303.11476 (2023).
