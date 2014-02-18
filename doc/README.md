A short user-manual for using RRR
=================================

The provided piece of code provides a way of robustifying the back-end against incorrect loop closures that 
may arise due to preceptual aliasing in the SLAM problem. 

Currently, the code uses g2o[1] as the back-end but support will be added soon for other SLAM solvers.


Parameters of the algorithm
---------------------------

The algorithm proceeds by finding topologically connect places and gathering them into clusters. 
For this, the algorithm needs a estimate of the __clustering threshold__. This thershold defines
the max distance between two loop closures, after which they will be considered to lie in different clusters. 

For maps in which nodes are introduced at a constant rate (such as maps build with laser odometry or 
with visual odometry where nodes are added at fixed time intervals), there is a known odometry rate. 
Similarly, loop closing with vision is carried out at a fixed rate (e.g. 1 Hz). 

In those cases, the clustering threshold can be using 10*odometry\_rate/loop\_closure_rate. For example,
an odometry rate of 5Hz with a 1Hz loop closing rate would  give a clustering threshold of 50 poses.

In cases, where poses are key-frames and loops are closed with these key-frame, a clustering threshold 
of 10 poses can be used. (If you have no idea how to set it, let it be 10).

Using the code in your project
------------------------------

The current version of the code can be used in two different ways:

* Reading from the file

  To test how RRR works on a give pose graph, a g2o file containting a 2D/3D SLAM problem can be directly read
  by RRR and correct loop closures can be determined. (See examples/RRR\_3D\_from\_file\_g2o.cpp)

* Already existing instance of _g2o::SparseOptimizer_

  For use in cases where you already have constructed a pose-graph held in memory in an instance of g2o::SparseOptimizer,
  this optimizer can be passed directly to RRR to reason over. Incorrect links can be detected and removed from the 
  original graph. (See examples/RRR\_2D\_optimizer\_g2o.cpp)
  
  

If you use this work, please cite our corresponding papers : 

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

[1]: http://www.openslam.org/g2o

