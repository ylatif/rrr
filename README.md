## Introduction

- UPDATE (23/06/2021) : Accomodated changes in g2o API.
- UPDATE (17/02/2014) : Complete code rewrite to allow extending to various SLAM back-ends <br>
- UPDATE : Added timestamps in datasets/B25b/timestamps because of different number of poses in Odometry and Ground Truth g2o files

This project contains the accompanying code for our RSS 2012 paper Robust Loop Closing over time

## Requirements

- g2o :(https://github.com/RainerKuemmerle/g2o)
- boost math libraries

## Installation

- g2o: Install from offical repo following instructions [here](https://github.com/RainerKuemmerle/g2o). This verions has been tested with the [20201223](https://github.com/RainerKuemmerle/g2o/releases/tag/20201223_git) release

- install boost maths (on ubuntu use the following )

  ```
  sudo apt-get install libboost-math-dev
  ```

- in the directory rrr/<br>

  ```
  mkdir build && cd build
  cmake ..
  make
  ```

  This will generate examples in the folder rrr/build/examples/

## Running the examples

The example folder contains two executables.

- RRR_2D_optimizer_g2o : needs a 2D SLAM g2o file as input and generates rrr-solved.g2o as output.
- RRR_3D_from_disk_g2o : needs a 3D SLAM g2o file as input

The code has been restructured into more organized blocks. The main class of interest is include/RRR.hpp.
Further information on how to use this in your on project can be found in doc/

~~Kindly drop me an email at ylatif AT unizar DOT es in case something is not working.~~

Please open an new issue if you run into problems.

## Citing this work

If you use this work, please cite our corresponding paper :

@article{Latif-IJRR-13,<br>
author = {Latif, Yasir and Cadena, César and Neira, José},<br>
title = {Robust loop closing over time for pose graph SLAM},<br>
volume = {32}, <br>
number = {14}, <br>
pages = {1611-1626},<br>
year = {2013}, <br>
doi = {10.1177/0278364913498910},<br>
URL = {http://ijr.sagepub.com/content/32/14/1611.abstract},<br>
journal = {The International Journal of Robotics Research} <br>
}

@INPROCEEDINGS{Latif-RSS-12,<br>
author = {Y. Latif and C. Cadena and J. Neira},<br>
title = {{Robust Loop Closing Over Time}},<br>
booktitle = {Proceedings of Robotics: Science and Systems},<br>
year = {2012},<br>
address = {Sydney, Australia},<br>
month = {July}<br>
}
