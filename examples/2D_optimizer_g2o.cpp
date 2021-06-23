/*
 * test.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: yasir
 */

#include "include/RRR.hpp"

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"

#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"

typedef RRR<G2O_Interface<
	g2o::VertexSE2, g2o::EdgeSE2>>
	RRR_2D_G2O;

typedef RRR<G2O_Interface<
	g2o::VertexSE3, g2o::EdgeSE3>>
	RRR_3D_G2O;

/*!
 * This example show how to apply RRR to already existing instance of g2o optimizer.
 * To simulate this, a graph is read into the optimizer and then a pointer of this
 * optimizer is passed to an instance of RRR.
 *
 *
 */

int main(int argc, char **argv)
{

	if (argc < 2)
	{
		std::cerr << "Please specify a graph file to read " << std::endl;
		std::cerr << argv[0] << " graph_file clusteringThreshold[default=50]" << std::endl;
		return -1;
	}

	int clusteringThreshold = 50;
	int nIter = 4;

	if (argc == 3)
	{
		clusteringThreshold = atoi(argv[2]);
	}
	std::cout << std::endl;
	std::cout << "-------------------------------------" << std::endl;
	std::cout << "  RRR: Robust Loop Closing over time   " << std::endl;
	std::cout << " Yasir Latif, Cesar Cadena, Jose Neira " << std::endl;
	std::cout << "      University of Zargoza, 2014      " << std::endl;
	std::cout << "-------------------------------------" << std::endl;
	std::cout << std::endl;

	std::cout << "Reading from file :" << argv[1] << std::endl;
	std::cout << "Clustering threshold (t_g) :" << clusteringThreshold << std::endl;

	/* Allocate a g2o optmizer :
	 * It is important that the solver be define in case we want to
	 * reason on a graph already in memory.
	 * */

	g2o::SparseOptimizer optimizer;

	auto linearSolver = g2o::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
	linearSolver->setBlockOrdering(false);
	auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));
	g2o::OptimizationAlgorithmGaussNewton *solverGauss = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));

	optimizer.setAlgorithm(solverGauss);

	/* load the graph file in the optimizer */
	optimizer.load(argv[1]);

	/* Initialized RRR with the parameters defined */
	RRR_2D_G2O rrr(clusteringThreshold, nIter);

	/* Pass the current optimizer pointer to rrr */
	rrr.setOptimizer(&optimizer);

	/* Find loop closure that are consistent
	 * If the function is passed a bool variable with value true,
	 * it will automatically elimiate all the wrong loops from the
	 * original optimizer. Otherwise, the function removeIncorrectLoops()
	 * can be called to do the same.
	 */
	rrr.robustify();

	/**
	 * If we didn't remove the wrong loop closures earlier, remove them now
	 */
	rrr.removeIncorrectLoops();

	/*
	 * This will write a graph with only the correct loop closures, even when we have not
	 * elimiated incorrect ones using one of the methods above.
	 * */

	std::cout << std::endl;
	std::cout << "Output written to rrr-solved.g2o" << std::endl;

	rrr.write("rrr-solved.g2o");

	/**
	 * Since we have removed the incorrect ones, this file would be the same
	 * as the one above.
	 */

	//optimizer.save("g2o-saved.g2o");

	return 0;
}
