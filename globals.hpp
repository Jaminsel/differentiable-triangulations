#pragma once
#include "types.hpp"
#include <mutex>

extern std::mutex printMutex;
extern std::unordered_map<VertexHandle, double> vertexDataMap;
extern std::unordered_map<VertexHandle, int> vertexIndexMap;
extern std::unordered_map<std::array<VertexHandle, 3>, int, VertexHandleArrayHash> faceIndexMap;
extern std::unordered_map<Flip, int, FlipHash> flipIndexMap;
extern int triangulationSize;
extern int maximumDistance;
extern int max_iters;
extern double convergence_eps; 
extern double step_size;
extern bool variable_step_size;
extern bool showEdges;
extern bool showEdgeVisualization;
extern bool showWeightVisualization;
extern int alpha;
extern const char* BASE_SCREENSHOT_PATH;
