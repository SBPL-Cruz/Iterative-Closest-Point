#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "glm/glm.hpp"

using namespace std;

class PointcloudICP {
private:
    ifstream fp_in;
public:
	PointcloudICP(string filename);
	PointcloudICP(float* points, int num_poses, int step_size);
	~PointcloudICP();

	std::vector<glm::vec4>	points;
};
