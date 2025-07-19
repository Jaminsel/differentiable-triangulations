#include <iostream>
#include <fstream>
#include <unordered_set>
#include <vector>
#include <array>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <functional>

#include "types.hpp"
#include "globals.hpp"
#include "signed_distance_to_bisector_utils.hpp"
#include "visualization_utils.hpp"
#include "flip_utils.hpp"
#include "estimateErrorEnergy.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_2.h>
#include <CGAL/Weighted_point_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

#include "polyscope/polyscope.h"

#include <TinyAD/ScalarFunction.hh>
#include <TinyAD/Scalar.hh>
#include <TinyAD/Utils/NewtonDecrement.hh>
#include <TinyAD/Utils/NewtonDirection.hh>
#include <TinyAD/Utils/LineSearch.hh>
#include <TinyAD/Utils/LinearSolver.hh>
#include <TinyAD/Utils/Timer.hh>

#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Dense>

#include "libs/stb_image/stb_image.h"

#include <clipper2/clipper.h>

#include <mutex>
#include <random>

Eigen::MatrixXd loadImage(const char* imagepath, int& width, int& height, int& channels) {
    // Load image
    unsigned char *imgData = stbi_load(imagepath, &width, &height, &channels, 1); // 1 channel = grayscale
    if(imgData == NULL){
        std::cerr << "Bild konnte nicht geladen werden!\n";
        exit(1);
    }

    // Save in Matrix
    Eigen::MatrixXd imageMatrix(height, width);
    for (int y = 0; y < height; ++y){
        for(int x = 0; x < width; ++x){
            imageMatrix(height - 1 - y,x) = imgData[y*width+x] / 255.0; // Normalize to [0,1]
        }
    }

    stbi_image_free(imgData);
    return imageMatrix;
}

RegularTriangulation initTriangulation(){
    // Create a random number generator
    std::random_device rd;  // can be used to obtain a seed for the random number engine
    std::mt19937 gen(42); 
    std::uniform_real_distribution<> disWeight(0, 5);
    std::uniform_real_distribution<> disPosition(0.0, triangulationSize);

    std::vector<WeightedPoint> weightedPoints;

    // random Position: int steps + offset
    for (int y = 0; y <= triangulationSize; y++){
        for(int x = 0; x <= triangulationSize; x++){
            // add point (x,y) to triangulation and set initial weight to equal value (1) for every vertex
            // double randomWeight = std::rand() % (50 + 1) / 10; //lower + std::rand() % (upper - lower + 1)
            double randomWeight = disWeight(gen);
            double offsetX = 0;
            double offsetY = 0;
            if (x!=0 && x!=triangulationSize && y!=0 && y!=triangulationSize && (x+y) != triangulationSize) { 
                offsetX = (std::rand() % 5 - 2) * 0.3; 
                offsetY = (std::rand() % 5 - 2) * 0.3;
            }
            weightedPoints.push_back(WeightedPoint(K2::Point_2(x + offsetX, y + offsetY), randomWeight));
        }
    }

    // fully random Position
    // corners //disWeight(gen)
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, triangulationSize), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(triangulationSize, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(triangulationSize, triangulationSize), 1));
    // for(int i = 0; i< triangulationSize*triangulationSize-4; i++){
    //     weightedPoints.push_back(WeightedPoint(K2::Point_2(disPosition(gen), disPosition(gen)), 1));
    // }

    // 3-1 flip
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, triangulationSize), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(triangulationSize, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(triangulationSize, triangulationSize), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, 1.1), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, 1.9), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.1, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.1, 3), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.9, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(2, 3), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(3, 1.16), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(3, 1.98), 1));

    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.5, 2.3), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(2.2, 1), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0.8, 1), 1));

    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.5, 1.5), 0.5)); //collinear at w = 0.3

    // 2-2 flip
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, triangulationSize), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(triangulationSize, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(triangulationSize, triangulationSize), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, 1.1), 0.9));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(0, 1.9), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.1, 0), 0.9));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1, 1), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1, 2), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.1, 3), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(1.9, 0), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(2, 1), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(2, 2), 1.1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(2,3), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(3, 1.16), 1));
    // weightedPoints.push_back(WeightedPoint(K2::Point_2(3, 1.98), 1));

    RegularTriangulation triangulation(weightedPoints.begin(), weightedPoints.end());
    triangulation.is_valid();
    return triangulation;
}

Eigen::VectorXd clampVector (const Eigen::VectorXd& x){
    Eigen::VectorXd clampedX = x;
    for (int i = 0; i < x.size(); i += 4) { // Iterate over the vector in chunks of 3 (x, y, c for each vertex)
        clampedX(i) = std::clamp(x(i), 0.0, static_cast<double>(triangulationSize));            // Clamp x coordinates to [0, triangulationSize]
        clampedX(i + 1) = std::clamp(x(i + 1), 0.0, static_cast<double>(triangulationSize));    // Clamp y coordinates to [0, triangulationSize]
        clampedX(i + 2) = std::clamp(x(i + 2), 0.0, 10.0);                                       // Clamp weight to [0.0, 5.0]
        clampedX(i + 3) = std::clamp(x(i + 3), 0.0, 1.0);                                       // Clamp color to [0.0, 1.0]
    }
    return clampedX;
}

template <typename T>
T clampValue (T value, double min, double max){
    if(value < min) value = min;
    else if (value > max) value = max;
    return value;
}

void minimizeErrorEnergy(RegularTriangulation triangulation, const Eigen::MatrixXd& imageMatrix) {
    bool vertexHandleChange = false;
    bool nonFlipFaces = false;

    // set initial color
    std::mt19937 gen(42); // ransom seed: rd()
    std::uniform_real_distribution<> disWeight(0, 1);
    for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
        vertexDataMap[v] = 0.5;
        // vertexDataMap[v] = disWeight(gen);
        // 2-2 flip
        // if((v->point().x() + v->point().y()) == triangulationSize) vertexDataMap[v] = 0;
        // else vertexDataMap[v] = 1;
        // 3-1 flip
        // if(v->point().y() == 1.5 || v->point().y() == 1.5) vertexDataMap[v] = 0;
        // else vertexDataMap[v] = 1;
    }
    for (auto hv = triangulation.hidden_vertices_begin(); hv != triangulation.hidden_vertices_end(); ++hv) {
        vertexDataMap[hv] = 0.5; 
        // vertexDataMap[hv] = disWeight(gen);
        // 2-2 flip
        // if((hv->point().x() + hv->point().y()) == triangulationSize) vertexDataMap[hv] = 0;
        // else vertexDataMap[hv] = 1;
        // 3-1 flip
        // if(hv->point().y() == 1.5 || hv->point().y() == 1.5) vertexDataMap[hv] = 0;
        // else vertexDataMap[hv] = 1;
    }
    double energyWDT = 0.0;
    for(auto f = triangulation.finite_faces_begin(); f != triangulation.finite_faces_end(); ++f){
        energyWDT += estimateErrorEnergyWDT(Eigen::Vector2d(f->vertex(0)->point().x(), f->vertex(0)->point().y()), Eigen::Vector2d(f->vertex(1)->point().x(), f->vertex(1)->point().y()), Eigen::Vector2d(f->vertex(2)->point().x(), f->vertex(2)->point().y()), vertexDataMap[f->vertex(0)], vertexDataMap[f->vertex(1)], vertexDataMap[f->vertex(2)], imageMatrix);
    }

    // Polyscope ----------------------------------------------------------------------
    std::map<VertexHandle, size_t> vertexToIndex;
    polyscope::SurfaceMesh* mesh = initializeVisualization(triangulation); // general Visualization
    polyscope::SurfaceMesh* edgeVisualization;
    if(showEdgeVisualization) edgeVisualization = initializeEdgeVisualization(triangulation); 
    polyscope::PointCloud* pointCloud;
    if(showWeightVisualization) pointCloud = initializeWeightVisualization(triangulation);
    resetCamera();
    // polyscope::screenshot(std::string(BASE_SCREENSHOT_PATH) + "input_triangulation.png", true);
    // --------------------------------------------------------------------------------

    std::cout << "Iteration 0  Step size: " << step_size << "  Faces: " << triangulation.number_of_faces() << "  Vertices: " << triangulation.number_of_vertices() <<  "  Hidden vertices: " << triangulation.number_of_hidden_vertices() << std::endl; 

    // Projected Newton
    std::cout << "Starting Projected Newton" << std::endl;
    TinyAD::LinearSolver solver;
    double previous_error = 0.0;

    // csv file for saving optimization data 
    std::ofstream data_file("data.csv");
    data_file << "Iteration,Energy,EnergyWDT,Step_Size,Faces,Vertices,Hidden_Vertices\n"; //header
    data_file << "0" << "," << std::setprecision(15) << "0" << "," << energyWDT << "," << step_size << "," << triangulation.number_of_faces() << "," << triangulation.number_of_vertices() << "," << triangulation.number_of_hidden_vertices() << "\n";

    bool colorConverged = true;
    for (int i = 1; i < max_iters; ++i) {
        // std::cout << "Starting iteration: " << i << std::endl;
        std::vector<VertexHandle> vertexHandles;
        std::vector<std::array<VertexHandle, 3>> faceHandles;
        std::vector<Flip> flips;
        std::unordered_set<Flip, FlipHash> uniqueFlips;
        std::set<std::array<VertexHandle, 3>> nonCoveredFaces;
        std::vector<VertexHandle> includedVertexHandles;
        int includedTrianglesCounter = 0;

        {
            int index = 0;
            for (auto v = triangulation.finite_vertices_begin(); v != triangulation.finite_vertices_end(); ++v) {
                vertexHandles.push_back(v);
                vertexIndexMap[v] = index++;
                includedVertexHandles.push_back(v);
            }
            for (auto hv = triangulation.hidden_vertices_begin(); hv != triangulation.hidden_vertices_end(); ++hv) {
                vertexHandles.push_back(hv); 
                vertexIndexMap[hv] = index++; 

                includedVertexHandles.push_back(hv);
            }
        }

        // add all currently existing faces into a set
        for(auto f = triangulation.finite_faces_begin(); f != triangulation.finite_faces_end(); ++f){
            std::array<VertexHandle, 3> face = {f->vertex(0), f->vertex(1), f->vertex(2)};
            std::sort(face.begin(), face.end());
            nonCoveredFaces.insert(face);
        }

        // construct all current faces and all faces that are "one flip away"
        flipIndexMap.clear();
        int index = 0;
        int flip22 = 0;
        int flip31 = 0;
        int flip13 = 0;
        for (auto e = triangulation.finite_edges_begin(); e != triangulation.finite_edges_end(); ++e) {
            FaceHandle f = e->first; // face that contains e
            int i = e->second; // i ∈ {0, 1, 2} index of the opposite vertex in the triangle

            // Skip if this face or its neighbor is infinite (border edge)
            if (triangulation.is_infinite(f) || triangulation.is_infinite(f->neighbor(i))) {
                continue;
            }

            // retrieve the 4 relevant vertices
            VertexHandle v1 = f->vertex((i + 1) % 3); // edge vertex 1
            VertexHandle v2 = f->vertex((i + 2) % 3); // edge vertex 2
            VertexHandle v3 = f->vertex(i); // adjacent face 1 vertex 

            FaceHandle fs_neighbor = f->neighbor(i);
            int j = fs_neighbor->index(f);
            VertexHandle v4 = fs_neighbor->vertex(j); // adjacent face 2 vertex

            Flip flip;

            if(count_neighbors(triangulation, v1) == 3){
                if(is_corner_vertex(v1)) {
                    if(is_convex_quad(v1, v2, v3, v4)){
                        // 2-2 flip in corner
                        createFlip(flip, 2, v1, v2, v3, v4);
                    } else if (count_neighbors(triangulation, v2) == 3) {
                        // 3-1 flip, with one edge vertex (v1) being a corner
                        createFlip(flip, 3, v2, v1, v3, v4);

                    }
                    //else not flippable: skip that
                    continue;
                } else if(is_edge_vertex(v1)) {
                    // 2-2 flip with collinear points (2-1 flip)
                    createFlip(flip, 2, v1, v2, v3, v4);
                } else {
                    // 3-1 flip
                    createFlip(flip, 3, v1, v2, v3, v4);
                }
            } else if(count_neighbors(triangulation, v2) == 3){
                if(is_corner_vertex(v2)) {
                    if(is_convex_quad(v1, v2, v3, v4)){
                        // 2-2 flip in corner
                        createFlip(flip, 2, v1, v2, v3, v4);
                    } else if (count_neighbors(triangulation, v1) == 3) {
                        // 3-1 flip, with one edge vertex (v2) being a corner
                        createFlip(flip, 3, v1, v2, v3, v4);
                    }
                    //else not flippable: skip that
                    continue;
                } else if(is_edge_vertex(v2)) {
                    // 2-2 flip with collinear points (2-1 flip)
                    createFlip(flip, 2, v1, v2, v3, v4);
                } else {
                    // 3-1 flip
                    createFlip(flip, 3, v2, v1, v3, v4);
                }
            } else if (is_convex_quad(v1, v2, v3, v4)){
                // 2-2 flip
                createFlip(flip, 2, v1, v2, v3, v4);
            } else {
                // flip not possible: skip that 
                continue;
            }

            if(uniqueFlips.insert(flip).second){ // insert, if not already in set (second: true, if new element was inserted)
                flips.push_back(flip);
                flipIndexMap[flip] = index++;

                // construct currently existing faces and remove from nonCoveredFaces Set
                std::vector<std::array<VertexHandle, 3>> existingFaces;
                existingFaces.push_back({v1, v2, v3});
                existingFaces.push_back({v1, v2, v4});
                if(flip.flipKind == 3) existingFaces.push_back({flip.flipVertices[0], v3, v4});

                for(std::array<VertexHandle, 3> existingFace : existingFaces){
                    std::sort(existingFace.begin(), existingFace.end());
                    nonCoveredFaces.erase(existingFace);
                }

                if(flip.flipKind == 2) flip22++;
                if(flip.flipKind == 3) flip31++;
            }
        }

        // construct all hidden faces 
        for (auto hv = triangulation.hidden_vertices_begin(); hv != triangulation.hidden_vertices_end(); ++hv) {
            FaceHandle face = triangulation.locate(hv->point());
            Flip flip;
            createFlip(flip, 1, hv, face->vertex(0), face->vertex(1), face->vertex(2));

            if(uniqueFlips.insert(flip).second){
                flips.push_back(flip);
                flipIndexMap[flip] = index++;

                std::array<VertexHandle, 3> existingFace = {face->vertex(0), face->vertex(1), face->vertex(2)};
                std::sort(existingFace.begin(), existingFace.end());
                nonCoveredFaces.erase(existingFace);
                flip13++;
            }
        }

        // std::cout << "flips: " << flip13+flip22+flip31 << " (" << flip13 << ", " << flip22 << ", " << flip31 << ")" << std::endl;

        auto func = TinyAD::scalar_function<4>(vertexHandles); // 4: x-coordinate, y-coordinate, weight, color
        // Define the energy function over the faces
        func.add_elements<4>(flips, [&](auto& element) -> TINYAD_SCALAR_TYPE(element) {
            using T = TINYAD_SCALAR_TYPE(element);
            const Flip& flip = element.handle;
            std::vector<Eigen::Vector4<T>> vertices;

            // Collect vertex positions and colors
            for (int j = 0; j < 4; ++j) {
                VertexHandle vh = flip.flipVertices[j];
                Eigen::Vector4<T> vertexVariables = element.variables(vh);
                K2::Point_2 p = vh->point().point();

                // vertexVariables(0) = TinyAD::to_passive(vertexVariables(0)); // fix x
                // vertexVariables(1) = TinyAD::to_passive(vertexVariables(1)); // fix y
                // vertexVariables(3) = TinyAD::to_passive(vertexVariables(3)); // fix color
                // vertexVariables(2) = TinyAD::to_passive(vertexVariables(2)); // fix weight

                // fix corners
                if (p == K2::Point_2(0, 0) || p == K2::Point_2(0, triangulationSize) || 
                    p == K2::Point_2(triangulationSize, 0) || p == K2::Point_2(triangulationSize, triangulationSize)) {
                        vertexVariables(0) = TinyAD::to_passive(vertexVariables(0));
                        vertexVariables(1) = TinyAD::to_passive(vertexVariables(1));
                }
            
                vertices.push_back(vertexVariables);
            }

            assert(vertices.size() == 4);

            // get coordinates x, y of each vertex
            Eigen::Vector2<T> p0 = vertices[0].head<2>(); 
            Eigen::Vector2<T> p1 = vertices[1].head<2>(); 
            Eigen::Vector2<T> p2 = vertices[2].head<2>(); 
            Eigen::Vector2<T> p3 = vertices[3].head<2>(); 

            // get color c of each vertex
            T c0 = vertices[0][3];
            T c1 = vertices[1][3];
            T c2 = vertices[2][3];
            T c3 = vertices[3][3];

            // get weight w of each vertex
            T w0 = vertices[0][2];
            T w1 = vertices[1][2];
            T w2 = vertices[2][2];
            T w3 = vertices[3][2];

            Eigen::Vector3<T> lifted_p0 = lift_point(p0, w0);
            Eigen::Vector3<T> lifted_p1 = lift_point(p1, w1);
            Eigen::Vector3<T> lifted_p2 = lift_point(p2, w2);
            Eigen::Vector3<T> lifted_p3 = lift_point(p3, w3);

            // calculate inclusion score
            T inclusionScore = get_inclusion_score(lifted_p0, lifted_p1, lifted_p2, lifted_p3);

            // print inclusion score
            // std::cout << "Inclusion Score (C): " << inclusionScore << std::endl;

            if(flip.flipKind == 1){ // 1-3 flip
                // get face errors 
                T errorCurrentFace = estimateErrorEnergy(p1, p2, p3, c1, c2, c3, imageMatrix);
                T errorFlippedFace1 = estimateErrorEnergy(p0, p1, p2, c0, c1, c2, imageMatrix);
                T errorFlippedFace2 = estimateErrorEnergy(p0, p2, p3, c0, c2, c3, imageMatrix);
                T errorFlippedFace3 = estimateErrorEnergy(p0, p1, p3, c0, c1, c3, imageMatrix);

                if (!isfinite(errorCurrentFace) || !isfinite(errorFlippedFace1) || !isfinite(errorFlippedFace2) || !isfinite(errorFlippedFace3) || !isfinite(inclusionScore)) {
                    std::cout << "Invalid return: inclusionScore = " << inclusionScore << std::endl;
                }
                
                return inclusionScore * errorCurrentFace 
                + (1 - inclusionScore) *  errorFlippedFace1 
                + (1 - inclusionScore) *  errorFlippedFace2 
                + (1 - inclusionScore) *  errorFlippedFace3;

            } else if(flip.flipKind == 2){ // 2-2 flip
                // get face errors 
                T errorCurrentFace1 = estimateErrorEnergy(p0, p1, p2, c0, c1, c2, imageMatrix);
                T errorCurrentFace2 = estimateErrorEnergy(p0, p1, p3, c0, c1, c3, imageMatrix);
                T errorFlippedFace1 = estimateErrorEnergy(p2, p3, p0, c2, c3, c0, imageMatrix);
                T errorFlippedFace2 = estimateErrorEnergy(p2, p3, p1, c2, c3, c1, imageMatrix);

                if (!isfinite(errorCurrentFace1) || !isfinite(errorCurrentFace2) || !isfinite(errorFlippedFace1) || !isfinite(errorFlippedFace2) || !isfinite(inclusionScore)) {
                    std::cout << "Invalid return: inclusionScore = " << inclusionScore << std::endl;
                }

                return  inclusionScore *  errorCurrentFace1 
                + inclusionScore *  errorCurrentFace2 
                + (1 - inclusionScore) * errorFlippedFace1 
                + (1 - inclusionScore) * errorFlippedFace2;
                    
            } else if(flip.flipKind == 3){ // 3-1 flip
                // get face errors 
                T errorFlippedFace = estimateErrorEnergy(p1, p2, p3, c1, c2, c3, imageMatrix);
                T errorCurrentFace1 = estimateErrorEnergy(p0, p1, p2, c0, c1, c2, imageMatrix);
                T errorCurrentFace2 = estimateErrorEnergy(p0, p2, p3, c0, c2, c3, imageMatrix);
                T errorCurrentFace3 = estimateErrorEnergy(p0, p1, p3, c0, c1, c3, imageMatrix);

                if (!isfinite(errorFlippedFace) || !isfinite(errorCurrentFace1) || !isfinite(errorCurrentFace2) || !isfinite(errorCurrentFace3) || !isfinite(inclusionScore)) {
                    std::cout << "Invalid return: inclusionScore = " << inclusionScore << std::endl;
                }

                return (1 - inclusionScore) * errorFlippedFace 
                + inclusionScore *  errorCurrentFace1 
                + inclusionScore *  errorCurrentFace2 
                + inclusionScore *  errorCurrentFace3;
            } else {
                std::cout << "This should not happen!!! Non valid flipKind in func.add_elements!!" << std::endl;
                return 0;
            }
        });

        // Initialize x vector from vertex positions
        Eigen::VectorXd x = func.x_from_data([&](VertexHandle v) {
            if(!vertexDataMap[v] && vertexDataMap[v] != 0) std::cout << "VertexDataMap Error in funx.x_from_data in main.cpp: " << v->point() << " color: " << vertexDataMap[v] << std::endl;
            return Eigen::Vector4d(v->point().x(), v->point().y(), v->point().weight(), vertexDataMap[v]);
        });
        auto [f, g] = func.eval_with_gradient(x);
        double current_g_norm = g.norm();
        if (current_g_norm < convergence_eps) break;

        if(i > 1){
            if(variable_step_size){
                if(f > (1.5 * previous_error)) step_size *= 0.5;
                else if(f > previous_error) step_size *= 0.7;
                else {
                    double ratio = std::min(previous_error / f, 4.0);
                    step_size *= 0.8 + 0.2 * ratio;
                }
                step_size = std::clamp(step_size, 1e-10, 0.2);
            }
        }
        previous_error = f;

        x -= step_size * g; // Gradientenabstieg
        x = clampVector(x);

        // calculate WDT loss
        energyWDT = 0.0;
        for(auto f = triangulation.finite_faces_begin(); f != triangulation.finite_faces_end(); ++f){
            energyWDT += estimateErrorEnergyWDT(Eigen::Vector2d(f->vertex(0)->point().x(), f->vertex(0)->point().y()), Eigen::Vector2d(f->vertex(1)->point().x(), f->vertex(1)->point().y()), Eigen::Vector2d(f->vertex(2)->point().x(), f->vertex(2)->point().y()), vertexDataMap[f->vertex(0)], vertexDataMap[f->vertex(1)], vertexDataMap[f->vertex(2)], imageMatrix);
        }

        // Update triangulation
        func.x_to_data(x, [&](VertexHandle v, const Eigen::Vector4d& p){
            WeightedPoint newVertex = WeightedPoint(K2::Point_2(p[0], p[1]), p[2]);
            triangulation.remove(v);
            VertexHandle new_v = triangulation.insert(newVertex);
            vertexDataMap[new_v] = p[3]; 
            if (v != new_v){
                std::cout << "Vertex Handle changed !!!" << std::endl;
                std::cout << "old vertex: " << v->point().point() << std::endl;
                std::cout << "new vertex: " << new_v->point().point() << std::endl;
                vertexHandleChange = true;
            }
        });

        if(!triangulation.is_valid()) std::cout << "Triangulation is not valid !" << std::endl;

        
        // Polyscope ----------------------------------------------------------------------
        if(showWeightVisualization) updateWeightVisualization(pointCloud, triangulation);
        updateVisualization(mesh, triangulation);
        if(showEdgeVisualization) updateEdgeVisualization(edgeVisualization, triangulation);
        resetCamera();
        // --------------------------------------------------------------------------------

        if(i%10 == 1 || i==2){
            std::cout   << "Iteration " << i << "  Energy: " << f << "  Energy WDT: " << energyWDT << 
                "  Step size: " << step_size <<
                "  Faces: " << triangulation.number_of_faces() <<
                "  Vertices: " << triangulation.number_of_vertices() << 
                "  Hidden vertices: " << triangulation.number_of_hidden_vertices() << 
            std::endl; 
        }

        // Write to csv file 
        data_file << i-1 << ","
            << std::setprecision(15) << f << "," 
            << energyWDT << ","
            << step_size << ","
            << triangulation.number_of_faces() << ","
            << triangulation.number_of_vertices() << ","
            << triangulation.number_of_hidden_vertices() << "\n";
    }
    // polyscope::screenshot(std::string(BASE_SCREENSHOT_PATH) + "final_iteration.png", true);
    // std::cout << "Vertex Handles changed: " << vertexHandleChange << std::endl;
    // std::cout << "Non flippable Faces: " << nonFlipFaces << std::endl;
}

int main() {
    polyscope::view::style = polyscope::view::NavigateStyle::Planar;
    polyscope::view::projectionMode = polyscope::ProjectionMode::Orthographic;
    polyscope::init();

    int width, height, channels;
    const char* imagepath = "../data/Gradient2.png"; 
    Eigen::MatrixXd image = loadImage(imagepath, width, height, channels);
    // std::cout << "Image loaded with dimensions: " << height << "x" << width << std::endl;

    RegularTriangulation triangulation = initTriangulation();

    minimizeErrorEnergy(triangulation, image); 

    polyscope::show();
    return 0;
}