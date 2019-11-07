#pragma once

#include <stdio.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/device_vector.h>
#include <cuda.h>
#include <cmath>
#include <vector>
#include <iostream>
#include "kdtree.hpp"

using milli = std::chrono::milliseconds;

// Controls for ICP implementation
#define KD_TREE_SEARCH 1
#define INITIAL_ROT 0


// LOOK-2.1 potentially useful for doing grid-based neighbor search
#ifndef imax
#define imax( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef imin
#define imin( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

#ifndef clamp
#define clamp(x, lo, hi) (x < lo) ? lo : (x > hi) ? hi : x
#endif

#ifndef wrap
#define wrap(x, lo, hi) (x < lo) ? x + (hi - lo) : (x > hi) ? x - (hi - lo) : x
#endif

#define checkCUDAErrorWithLine(msg) checkCUDAError(msg, __LINE__)

/*****************
* Configuration *
*****************/

/*! Block size used for CUDA kernel launch. */
#define blockSize 256
#define sharedMemorySize 65536

/*! Size of the starting area in simulation space. */
// #define scene_scale 50.0f
#define scene_scale 1.0f

class ICP 
{
	private:
		int sizeTarget;
		int sizeScene;
		int numObjects;

		glm::vec4 *dev_pos;
		glm::vec3 *dev_color;
		int *dev_dist;
		int *dev_pair;
		KDTree::Node *dev_kd;

		glm::vec4 *host_pos;
		int *host_dist;
		int *host_pair;
	public:
		ICP();
		void initSimulation(std::vector<glm::vec4> scene, std::vector<glm::vec4> target, KDTree::Node *kd);
		void endSimulation();

		// Base ICP implementation obtained from http://ais.informatik.uni-freiburg.de/teaching/ss12/robotics/slides/17-icp.pdf
		void stepCPU();
		void stepGPU(glm::mat4& total_transform);
		void checkConvergence(int thresh);
		
		void copyPointsToVBO(float *vbodptr_positions, float *vbodptr_velocities);

		void unitTest();

		bool iterateGPU();
};
