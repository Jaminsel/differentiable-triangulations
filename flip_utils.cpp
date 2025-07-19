#include "flip_utils.hpp"

// distance is positive if p is above the plane
double get_vertical_distance_to_plane(const Eigen::Vector3d& p, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3){
        // Get three vectors on the plane
        Eigen::Vector3d u = v2 - v1;
        Eigen::Vector3d v = v3 - v1;
    
        // Compute the normal to the plane
        Eigen::Vector3d n = u.cross(v);
        
        double a = n.x();
        double b = n.y();
        double c = n.z();

        // Plane equation: ax + by + cz + d = 0
        double d = -n.dot(v1);  // since v1 lies on the plane

        if (std::abs(c) < 1e-12) {
            throw std::runtime_error("Plane is vertical — undefined vertical distance.");
        }

        // Compute z on the plane at p.x(), p.y()
        double z_plane = -(a * p.x() + b * p.y() + d) / c;

        // Return vertical distance (signed)
        return p.z() - z_plane;
}

int count_neighbors(const RegularTriangulation& triangulation, const VertexHandle& v) {
    int neighbors = 0;

    RegularTriangulation::Vertex_circulator vcirc = triangulation.incident_vertices(v);
    RegularTriangulation::Vertex_circulator done = vcirc;

    if (vcirc != nullptr) {
        while (true) {
            if (!triangulation.is_infinite(vcirc)) {
                neighbors++;
            }
            ++vcirc;
            if (vcirc == done) break;
        } 
    }
    return neighbors;
}

// checks if the 4 vertices form a convex quadrilateral
// assumes that v1 and v2 are opposites in the quad
bool is_convex_quad(const VertexHandle& v1, const VertexHandle& v2, const VertexHandle& v3, const VertexHandle& v4) {
    CGAL::Segment_2<K2> current_edge(v1->point().point(), v2->point().point());
    CGAL::Segment_2<K2> potential_flip(v3->point().point(), v4->point().point());
    return CGAL::do_intersect(current_edge, potential_flip);
}

bool is_corner_vertex(const VertexHandle& v){
    K2::Point_2 p = v->point().point();
    if (p == K2::Point_2(0, 0) || p == K2::Point_2(0, triangulationSize) || 
        p == K2::Point_2(triangulationSize, 0) || p == K2::Point_2(triangulationSize, triangulationSize)) {
        return true;
    }
    return false;
}

// returns true if edge vertex
// returns false if corner vertex
bool is_edge_vertex(const VertexHandle& v){
    K2::Point_2 p = v->point().point();
    if(is_corner_vertex(v)) return false;

    if (p.x() == 0 || p.x() == triangulationSize || p.y() == 0 || p.y() == triangulationSize ) {
        return true;
    }
    return false;
}

// sort Vertexhandles that are order independet in their meaning
void createFlip(Flip& flip, int flipKind, const VertexHandle& v1, const VertexHandle& v2, const VertexHandle& v3, const VertexHandle& v4){
    flip.flipKind = flipKind;

    if(flipKind == 1 || flipKind == 3){
        flip.flipVertices[0] = v1;
        // sort the rest
        std::array<VertexHandle, 3> vertices = {v2, v3, v4};
        std::sort(vertices.begin(), vertices.end());
        flip.flipVertices[1] = vertices[0];
        flip.flipVertices[2] = vertices[1];
        flip.flipVertices[3] = vertices[2];
    } else if(flipKind == 2){
        // sort first two vertices
        std::array<VertexHandle, 2> edgeVertices = {v1, v2};
        std::sort(edgeVertices.begin(), edgeVertices.end());
        flip.flipVertices[0] = edgeVertices[0];
        flip.flipVertices[1] = edgeVertices[1];
        // sort last two vertices
        std::array<VertexHandle, 2> adjacentVertices = {v3, v4};
        std::sort(adjacentVertices.begin(), adjacentVertices.end());
        flip.flipVertices[2] = adjacentVertices[0];
        flip.flipVertices[3] = adjacentVertices[1];
    } else {
        std::cout << "This should not happen - createFlip in flip_utils.cpp" << std::endl;
    }
}