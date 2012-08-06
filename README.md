Introduction
------------

This project contains the accompanying code for our RSS 2012 paper Robust Loop Closing over time

Requirements
------------

- g2o http://www.openslam.org/g2o
- boost math libraries

Installation
-------------

- First install g2o (http://www.openslam.org/g2o) following the instructions given
- install boost maths ( On ubuntu : sudo apt-get install libboost-math1.42-dev)
- in the build folder of this project do "make" (Please check if library paths are correct) 
- this will generate an executable by the name "rrr"
- follow the instruction in the readme file in the build folder for running the example

If you use this work, please cite our corresponding paper : 

@INPROCEEDINGS{Latif-RSS-12,<br>
  author = {Y. Latif and C. Cadena and J. Neira},<br>
  title = {{Robust Loop Closing Over Time}},<br>
  booktitle = {Proceedings of Robotics: Science and Systems},<br>
  year = {2012},<br>
  address = {Sydney, Australia},<br>
  month = {July}<br>
}