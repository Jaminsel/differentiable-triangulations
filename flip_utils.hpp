#pragma once 
#include "types.hpp"
#include "globals.hpp"
#include <Eigen/Dense>
#include <iostream>

template<typename T>
Eigen::Vector3<T> lift_point(const Eigen::Vector2<T>& p, T w) {
    return Eigen::Vector3<T>(p[0], p[1], p[0] * p[0] + p[1] * p[1] - w);
}

double get_vertical_distance_to_plane(const Eigen::Vector3d& p, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3);

int count_neighbors(const RegularTriangulation& triangulation, const VertexHandle& v);

bool is_convex_quad(const VertexHandle& v1, const VertexHandle& v2, const VertexHandle& v3, const VertexHandle& v4);

bool is_corner_vertex(const VertexHandle& v);

bool is_edge_vertex(const VertexHandle& v);

void createFlip(Flip& flip, int flipKind, const VertexHandle& v1, const VertexHandle& v2, const VertexHandle& v3, const VertexHandle& v4);

// distance is positive if p is above the plane
template<typename T>
T get_vertical_distance_to_plane_Diff(const Eigen::Vector3<T>& p, const Eigen::Vector3<T>& v1, const Eigen::Vector3<T>& v2, const Eigen::Vector3<T>& v3){
    // Get three vectors on the plane
    Eigen::Vector3<T> u = v2 - v1;
    Eigen::Vector3<T> v = v3 - v1;

    // Compute the normal to the plane
    Eigen::Vector3<T> n = u.cross(v);
    
    T a = n.x();
    T b = n.y();
    T c = n.z();

    // Plane equation: ax + by + cz + d = 0
    T d = -n.dot(v1);  // since v1 lies on the plane

    if (c < 1e-12 && c > -1e-12) { // abs(c) < 1e-12
        throw std::runtime_error("Plane is vertical — undefined vertical distance.");
    }

    // Compute z on the plane at p.x(), p.y()
    T z_plane = -(a * p.x() + b * p.y() + d) / c;

    // Return vertical distance (signed)
    T distance = p.z() - z_plane;

    if (!isfinite(distance)) std::cout << "oh no! Non-finite distance detected: " << TinyAD::to_passive(distance) <<
        " points: " << TinyAD::to_passive(p) << ", " << TinyAD::to_passive(v1) << ", " << TinyAD::to_passive(v2) << ", " << TinyAD::to_passive(v3) << std::endl;

    return distance;
}

template<typename T>
T get_inclusion_score(const Eigen::Vector3<T>& v1, const Eigen::Vector3<T>& v2, const Eigen::Vector3<T>& v3, const Eigen::Vector3<T>& v4){
    std::array<Eigen::Vector3<T>, 4> vertices = {v1, v2, v3, v4};

    // extract x,y - coordinates for collinearity check
    std::array<Eigen::Vector2d, 4> passiveVertices = {
        TinyAD::to_passive(v1).head<2>(),
        TinyAD::to_passive(v2).head<2>(),
        TinyAD::to_passive(v3).head<2>(),
        TinyAD::to_passive(v4).head<2>()
    };

    T score = 0;
    int scoreCounter = 0;

    for(int i = 0; i < 4; i++) { // iterate over all points
        int a = (i+1)%4;
        int b = (i+2)%4;
        int c = (i+3)%4;

        if (notCollinear(passiveVertices[a], passiveVertices[b], passiveVertices[c])){ // check if the three ther points are collinear
            T distance = get_vertical_distance_to_plane_Diff(vertices[i], vertices[a], vertices[b], vertices[c]); // distance from point i to plane(a,b,c)
            T partialScore = distance_to_inclusion_score(distance);
            
            if(partialScore < 0.5) partialScore = 1 - partialScore;
            
            score += partialScore;
            scoreCounter ++;
        }
    }

    if(scoreCounter == 0) return score;

    return score / scoreCounter;
}

template<typename T>
T distance_to_inclusion_score(T distance){

    if (!isfinite(distance)) std::cout << "oh no! Non-finite distance detected: " << TinyAD::to_passive(distance) << std::endl;

    // T x = distance * distance / sigma;
    T x = clampValue(distance * alpha , -maximumDistance, maximumDistance); // clamp to prevent overflow

    // Shift and scale sigmoid so that: distance = 0 → score = 0.5
    T s = T(1.0) / (T(1.0) + exp(-x));

    // std::cout << "distance & Inclusion Score " << TinyAD::to_passive(distance) << ", " << TinyAD::to_passive(s) << std::endl;

    return s;
}

// checks if 3 points do not lie on one line
inline bool notCollinear(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    double cross = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
    const double epsilon = 1e-10;
    return std::abs(cross) > epsilon;
}
