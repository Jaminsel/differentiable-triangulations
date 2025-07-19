#include "globals.hpp"
#include "types.hpp"
#include "visualization_utils.hpp"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include <iostream>
#include <mutex>

static std::mutex visualizationMutex;
float spacing = triangulationSize * 0.1f;

void resetCamera(){
    polyscope::view::resetCameraToHomeView();
    // polyscope::view::lookAt(
    //     glm::vec3(triangulationSize/2.0, 2 * triangulationSize, 2 * triangulationSize), // Camera position
    //     glm::vec3(triangulationSize/2.0, 0, 0), // Look-at target
    //     glm::vec3(0.0, 1.0, 0.0) // Up vector
    // );
}

polyscope::SurfaceMesh* initializeVisualization(const RegularTriangulation& triangulation) {
    std::cout << "Visualizing for the first time" << std::endl;
    std::lock_guard<std::mutex> lock(visualizationMutex);
    
    std::vector<std::array<double, 2>> vertices;
    std::vector<std::array<size_t, 3>> faces;
    std::vector<std::array<double, 3>> vertexColors;
    std::map<VertexHandle, size_t> vertexToIndex;

    size_t index = 0;

    for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
        vertices.push_back({v->point().x(), v->point().y()});
        vertexToIndex[v] = index++;
        double initialColor = vertexDataMap[v];
        vertexColors.push_back({initialColor, initialColor, initialColor});
    }

    for (auto f = triangulation.finite_faces_begin(); f != triangulation.finite_faces_end(); ++f) {
        faces.push_back({vertexToIndex[f->vertex(0)], vertexToIndex[f->vertex(1)], vertexToIndex[f->vertex(2)]});
    }

    polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh2D("Weighted Delaunay Triangulation", vertices, faces);
    mesh->addVertexColorQuantity("Grayscale", vertexColors)
        ->setEnabled(true);
    if(showEdges) {
        mesh->setEdgeWidth(1.0);
        mesh->setEdgeColor({0., 0., 0.});
    }
    mesh->setMaterial("flat"); 

    polyscope::frameTick();

    return mesh;
}

void updateVisualization(polyscope::SurfaceMesh* mesh,
    const RegularTriangulation& triangulation) {

    std::lock_guard<std::mutex> lock(visualizationMutex);
    
    std::vector<std::array<double, 2>> vertices;
    std::vector<std::array<size_t, 3>> faces;
    std::vector<std::array<double, 3>> vertexColors;
    std::map<VertexHandle, size_t> vertexToIndex;

    int index = 0;
    for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
        vertices.push_back({v->point().x(), v->point().y()});
        vertexToIndex[v] = index++;
        if(!vertexDataMap[v] && vertexDataMap[v] != 0) std::cout << "VertexDataMap Error in Visualizations_utils.cpp: " << v->point() << " color: " << vertexDataMap[v] << std::endl;
        vertexColors.push_back({vertexDataMap[v], vertexDataMap[v], vertexDataMap[v]});
    }

    for (auto f = triangulation.finite_faces_begin(); f != triangulation.finite_faces_end(); ++f) {
        faces.push_back({vertexToIndex[f->vertex(0)], vertexToIndex[f->vertex(1)], vertexToIndex[f->vertex(2)]});
    } 

    polyscope::removeSurfaceMesh("Weighted Delaunay Triangulation");

    mesh = polyscope::registerSurfaceMesh2D("Weighted Delaunay Triangulation", vertices, faces);

    mesh->removeAllQuantities();
    mesh->addVertexColorQuantity("Grayscale", vertexColors)
        ->setEnabled(true);
    if(showEdges) mesh->setEdgeWidth(1.0);

    polyscope::requestRedraw();
    polyscope::frameTick();
}

polyscope::SurfaceMesh* initializeEdgeVisualization(const RegularTriangulation& triangulation) {
    std::cout << "Visualizing for the first time" << std::endl;
    std::lock_guard<std::mutex> lock(visualizationMutex);
    
    std::vector<std::array<double, 2>> vertices;
    std::vector<std::array<size_t, 3>> faces;
    std::map<VertexHandle, size_t> vertexToIndex;

    size_t index = 0;

    for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
        vertices.push_back({v->point().x(), v->point().y()});
        vertexToIndex[v] = index++;
    }

    for (auto f = triangulation.finite_faces_begin(); f != triangulation.finite_faces_end(); ++f) {
        faces.push_back({vertexToIndex[f->vertex(0)], vertexToIndex[f->vertex(1)], vertexToIndex[f->vertex(2)]});
    }

    polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh2D("Edge Visualization", vertices, faces);
    mesh->setEdgeWidth(1.0);
    mesh->setEdgeColor({0., 0., 0.});
    mesh->setMaterial("flat"); 
    mesh->setSurfaceColor({1, 1, 1});

    glm::mat4x4 transform = glm::translate(glm::mat4x4(1.0f), glm::vec3(triangulationSize + spacing, 0.0f, 0.0f));
    mesh->setTransform(transform); 

    polyscope::frameTick();

    return mesh;
}

void updateEdgeVisualization(polyscope::SurfaceMesh* mesh,
    const RegularTriangulation& triangulation) {

    std::lock_guard<std::mutex> lock(visualizationMutex);
    
    std::vector<std::array<double, 2>> vertices;
    std::vector<std::array<size_t, 3>> faces;
    std::map<VertexHandle, size_t> vertexToIndex;

    int index = 0;
    for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
        vertices.push_back({v->point().x(), v->point().y()});
        vertexToIndex[v] = index++;
    }

    for (auto f = triangulation.finite_faces_begin(); f != triangulation.finite_faces_end(); ++f) {
        faces.push_back({vertexToIndex[f->vertex(0)], vertexToIndex[f->vertex(1)], vertexToIndex[f->vertex(2)]});
    } 

    polyscope::removeSurfaceMesh("Edge Visualization");

    mesh = polyscope::registerSurfaceMesh2D("Edge Visualization", vertices, faces);

    mesh->removeAllQuantities();
    mesh->setEdgeWidth(1.0);
    mesh->setEdgeColor({0., 0., 0.});
    mesh->setMaterial("flat"); 
    mesh->setSurfaceColor({1, 1, 1});

    glm::mat4x4 transform = glm::translate(glm::mat4x4(1.0f), glm::vec3(triangulationSize + spacing, 0.0f, 0.0f));
    mesh->setTransform(transform); 

    polyscope::requestRedraw();
    polyscope::frameTick();
}

void visualizeIncludedFaces(
    std::vector<VertexHandle> includedVertexHandles,
    polyscope::SurfaceMesh* includedMesh,
    std::vector<std::array<VertexHandle, 3>>& includedFaces,
    polyscope::SurfaceMesh* almostIncludedMesh,
    std::vector<std::array<VertexHandle, 3>> almostIncludedFaces) {

    std::lock_guard<std::mutex> lock(visualizationMutex);
    
    std::vector<std::array<size_t, 3>> includedFacesPolyscope;
    std::vector<std::array<double, 2>> includedVertices;
    std::map<VertexHandle, size_t> includedVertexToIndex;
    std::vector<std::array<size_t, 3>> almostIncludedFacesPolyscope;

    // included faces
    int index = 0;
    for (auto v : includedVertexHandles) {
        includedVertices.push_back({v->point().x(), v->point().y()});
        includedVertexToIndex[v] = index++;
    }

    for (auto f : includedFaces){
        includedFacesPolyscope.push_back({includedVertexToIndex[f[0]], includedVertexToIndex[f[1]], includedVertexToIndex[f[2]]});
    }

    if (includedMesh != nullptr) {
        polyscope::removeSurfaceMesh("Included Faces");
    }

    includedMesh = polyscope::registerSurfaceMesh2D("Included Faces", includedVertices, includedFacesPolyscope);
    includedMesh->setEnabled(true);
    includedMesh->setTransparency(1.0);
    includedMesh->setEdgeWidth(1.0);
    includedMesh->setSurfaceColor(glm::vec3{1.0f, 0.0f, 1.0f});

    glm::mat4x4 transform = glm::translate(glm::mat4x4(1.0f), glm::vec3(triangulationSize + spacing, 0.0f, 0.0f));
    includedMesh->setTransform(transform); 

    // almost included faces
    for (auto f : almostIncludedFaces){
        almostIncludedFacesPolyscope.push_back({includedVertexToIndex[f[0]], includedVertexToIndex[f[1]], includedVertexToIndex[f[2]]});
    }

    if (almostIncludedMesh != nullptr) {
        polyscope::removeSurfaceMesh("Almost included Faces");
    }

    almostIncludedMesh = polyscope::registerSurfaceMesh2D("Almost included Faces", includedVertices, almostIncludedFacesPolyscope);
    almostIncludedMesh->setEnabled(true);
    almostIncludedMesh->setTransparency(0.6);
    almostIncludedMesh->setEdgeWidth(1.0);
    almostIncludedMesh->setEdgeColor(glm::vec3{1.0f, 0.5f, 0.0f});
    almostIncludedMesh->setSurfaceColor(glm::vec3{1.0f, 0.5f, 0.0f});
    almostIncludedMesh->setTransform(transform); 

    // polyscope::view::resetCameraToHomeView();
    polyscope::requestRedraw();
    polyscope::frameTick();
}

void visualizeInclusionScore(
    std::vector<VertexHandle> includedVertexHandles,
    polyscope::CurveNetwork* edgeMesh,
    std::vector<std::array<VertexHandle, 3>>& includedFaces,
    std::vector<std::array<VertexHandle, 3>> almostIncludedFaces) {

    std::lock_guard<std::mutex> lock(visualizationMutex);

    std::vector<std::array<size_t, 2>> edgeIndices;
    std::vector<std::array<double, 3>> edgeColors;
    // std::vector<double> scalarValues;
    std::vector<std::array<double, 2>> includedVertices;
    std::vector<glm::vec3> vertices3d;
    std::map<VertexHandle, size_t> includedVertexToIndex;

    std::vector<std::array<size_t, 3>> faces = {};

    // included faces
    int index = 0;
    for (auto v : includedVertexHandles) {
        includedVertices.push_back({v->point().x(), v->point().y()});
        vertices3d.push_back(glm::vec3(v->point().x(), v->point().y(), 0.0f));
        includedVertexToIndex[v] = index++;
    }

    for (auto f : includedFaces){
        edgeIndices.push_back({includedVertexToIndex[f[0]], includedVertexToIndex[f[1]]});
        edgeIndices.push_back({includedVertexToIndex[f[1]], includedVertexToIndex[f[2]]});
        edgeIndices.push_back({includedVertexToIndex[f[2]], includedVertexToIndex[f[0]]});
        edgeColors.push_back(std::array<double, 3>{1.0, 0.0, 1.0});
        edgeColors.push_back(std::array<double, 3>{1.0, 0.0, 1.0});
        edgeColors.push_back(std::array<double, 3>{1.0, 0.0, 1.0});
    }

    for (auto f : almostIncludedFaces){
        edgeIndices.push_back({includedVertexToIndex[f[0]], includedVertexToIndex[f[1]]});
        edgeIndices.push_back({includedVertexToIndex[f[1]], includedVertexToIndex[f[2]]});
        edgeIndices.push_back({includedVertexToIndex[f[2]], includedVertexToIndex[f[0]]});
        edgeColors.push_back(std::array<double, 3>{1.0, 0.5, 0.0});
        edgeColors.push_back(std::array<double, 3>{1.0, 0.5, 0.0});
        edgeColors.push_back(std::array<double, 3>{1.0, 0.5, 0.0});
    }

    if (edgeMesh != nullptr) {
        polyscope::removeCurveNetwork("Inclusion Score");
    }

    edgeMesh = polyscope::registerCurveNetwork("Inclusion Score", vertices3d, edgeIndices);
    edgeMesh->removeAllQuantities();
    edgeMesh->setEnabled(true);
    edgeMesh->setTransparency(1.0);
    edgeMesh->addEdgeColorQuantity("Edge Type", edgeColors)
            ->setEnabled(true);

    glm::mat4x4 transform = glm::translate(glm::mat4x4(1.0f), glm::vec3(triangulationSize + spacing, -triangulationSize - spacing, 0.0f));
    edgeMesh->setTransform(transform); 

    polyscope::requestRedraw();
    polyscope::frameTick();
}

polyscope::PointCloud* initializeWeightVisualization(const RegularTriangulation& triangulation) {

    std::lock_guard<std::mutex> lock(visualizationMutex);

    std::vector<std::array<double, 3>> vertices; // 3D version
    std::vector<double> weights;

    for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
        vertices.push_back({v->point().x(), v->point().y(), 0.0}); // 2D -> 3D
        weights.push_back(v->point().weight()); 
    }

    for (auto hv = triangulation.hidden_vertices_begin(); hv != triangulation.hidden_vertices_end(); ++hv) {
        vertices.push_back({hv->point().x(), hv->point().y(), 0.0}); 
        weights.push_back(hv->point().weight()); 
    }
    
    polyscope::removePointCloud("Vertex Weights");

    polyscope::PointCloud* pointCloud = polyscope::registerPointCloud("Vertex Weights", vertices);

    std::vector<glm::vec3> blackColors(vertices.size(), glm::vec3(0.0f, 0.0f, 0.0f));
    pointCloud->addColorQuantity("Uniform Black", blackColors)->setEnabled(true);
    
    pointCloud->setMaterial("flat"); 

    pointCloud->addScalarQuantity("Weights", weights)
        ->setEnabled(false);
    pointCloud->setPointRadiusQuantity("Weights", true);
    pointCloud->setPointRadius(0.01, true);

    glm::mat4x4 transform = glm::translate(glm::mat4x4(1.0f), glm::vec3(0.0f, -triangulationSize -spacing, 0.0f));
    pointCloud->setTransform(transform); 

    polyscope::requestRedraw();
    polyscope::frameTick();

    return pointCloud;
}

void updateWeightVisualization(polyscope::PointCloud* pointCloud,
    const RegularTriangulation& triangulation) {

    std::lock_guard<std::mutex> lock(visualizationMutex);

    std::vector<std::array<double, 3>> vertices; // 3D version
    std::vector<double> weights;

    for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
        vertices.push_back({v->point().x(), v->point().y(), 0.0}); // 2D -> 3D
        weights.push_back(v->point().weight()); 
    }

    for (auto hv = triangulation.hidden_vertices_begin(); hv != triangulation.hidden_vertices_end(); ++hv) {
        vertices.push_back({hv->point().x(), hv->point().y(), 0.0}); 
        weights.push_back(hv->point().weight()); 
    }

    polyscope::removePointCloud("Vertex Weights");

    pointCloud = polyscope::registerPointCloud("Vertex Weights", vertices);
    pointCloud->removeAllQuantities();

    std::vector<glm::vec3> blackColors(vertices.size(), glm::vec3(0.0f, 0.0f, 0.0f));
    pointCloud->addColorQuantity("Uniform Black", blackColors)->setEnabled(true);

    pointCloud->setMaterial("flat");  

    pointCloud->addScalarQuantity("Weights", weights)
        ->setEnabled(false);
    pointCloud->setPointRadiusQuantity("Weights", true);
    pointCloud->setPointRadius(0.01, true);

    glm::mat4x4 transform = glm::translate(glm::mat4x4(1.0f), glm::vec3(0.0f, -triangulationSize -spacing, 0.0f));
    pointCloud->setTransform(transform); 

    polyscope::requestRedraw();
    polyscope::frameTick();
}