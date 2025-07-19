// globals.cpp
#include "globals.hpp"
#include <mutex>

std::mutex printMutex;
std::unordered_map<VertexHandle, double> vertexDataMap;
std::unordered_map<VertexHandle, int> vertexIndexMap;
std::unordered_map<std::array<VertexHandle, 3>, int, VertexHandleArrayHash> faceIndexMap;
std::unordered_map<Flip, int, FlipHash> flipIndexMap;
int triangulationSize = 9;
int maximumDistance = 5;
int max_iters = 500;

double convergence_eps = 0;
double step_size = 1e-6; 
bool variable_step_size = false;

bool showEdges = true; 
bool showEdgeVisualization = true; // show mesh without colors
bool showWeightVisualization = true;

int alpha = 50; 

const char* BASE_SCREENSHOT_PATH = ""; //add path where screenshots should be saved