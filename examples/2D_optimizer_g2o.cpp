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


typedef RRR < G2O_Interface
				<
				g2o::VertexSE2, g2o::EdgeSE2
				>
			>
			RRR_2D_G2O;

typedef RRR < G2O_Interface
				<
				g2o::VertexSE3, g2o::EdgeSE3
				>
			>
			RRR_3D_G2O;


/*!
 * This example show how to apply RRR to already existing instance of g2o optimizer.
 * To simulate this, a graph is read into the optimizer and then a pointer of this
 * optimizer is passed to an instance of RRR.
 *
 *
 */

int main(int argc, char** argv)
{

	if(argc < 2)
	{
		std::cerr<<"Please specify a graph file to read " <<std::endl;
		return -1;
	}

	int clusteringThreshold = 50;
	int nIter = 4;

	/* Allocate a g2o optmizer :
	 * It is important that the solver be define in case we want to
	 * reason on a graph already in memory.
	 * */

	g2o::SparseOptimizer optimizer;

	SlamLinearSolver* linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
	g2o::OptimizationAlgorithmGaussNewton* solverGauss   = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

	optimizer.setAlgorithm(solverGauss);

	/* load the graph file in the optimizer */
	optimizer.load(argv[1]);

	/* Initialized RRR with the parameters defined */
	RRR_2D_G2O rrr(clusteringThreshold,nIter);

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

	rrr.write("rrr-solved.g2o");

	/**
	 * Since we have removed the incorrect ones, this file would be the same
	 * as the one above.
	 */
	optimizer.save("g2o-saved.g2o");


	return 0;
}





