#pragma once
#include "types.hpp"
#include "globals.hpp"
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include <CGAL/Regular_triangulation_2.h>

extern std::mutex visualizationMutex;  // For thread-safe visualization updates
extern float spacing;

void resetCamera();

// Initialize visualization
polyscope::SurfaceMesh* initializeVisualization(
    const RegularTriangulation& triangulation
);

// Update visualization during optimization
void updateVisualization(
    polyscope::SurfaceMesh* mesh,
    const RegularTriangulation& triangulation
);

// Initialize visualization
polyscope::SurfaceMesh* initializeEdgeVisualization(
    const RegularTriangulation& triangulation
);

// Update visualization during optimization
void updateEdgeVisualization(
    polyscope::SurfaceMesh* mesh,
    const RegularTriangulation& triangulation
);

void visualizeIncludedFaces(
    std::vector<VertexHandle> includedVertexHandles,
    polyscope::SurfaceMesh* includedMesh,
    std::vector<std::array<VertexHandle, 3>>& includedFaces,
    polyscope::SurfaceMesh* almostIncludedMesh,
    std::vector<std::array<VertexHandle, 3>> almostIncludedFaces
);

void visualizeInclusionScore(
    std::vector<VertexHandle> includedVertexHandles,
    polyscope::CurveNetwork* edgeMesh,
    std::vector<std::array<VertexHandle, 3>>& includedFaces,
    std::vector<std::array<VertexHandle, 3>> almostIncludedFaces
);

polyscope::PointCloud* initializeWeightVisualization(
    const RegularTriangulation& triangulation
);

void updateWeightVisualization(
    polyscope::PointCloud* pointCloud,
    const RegularTriangulation& triangulation
);