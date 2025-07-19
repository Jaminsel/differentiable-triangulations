// shared types
#pragma once 

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>
#include <unordered_map>
#include <array>
#include <functional>
#include <TinyAD/ScalarFunction.hh>

// Kernel and triangulation types
typedef CGAL::Exact_predicates_inexact_constructions_kernel K1;
typedef CGAL::Simple_cartesian<double> K2;
typedef CGAL::Regular_triangulation_2<K2> RegularTriangulation;

// Basic handles
typedef RegularTriangulation::Weighted_point WeightedPoint;
typedef RegularTriangulation::Vertex_handle VertexHandle;
typedef RegularTriangulation::Face_handle FaceHandle;

// Custom hash for VertexHandle arrays 
struct VertexHandleArrayHash {
    std::size_t operator()(const std::array<VertexHandle, 3>& arr) const {
        std::size_t h1 = std::hash<VertexHandle>()(arr[0]);
        std::size_t h2 = std::hash<VertexHandle>()(arr[1]);
        std::size_t h3 = std::hash<VertexHandle>()(arr[2]);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct EdgeHash {
    std::size_t operator()(const RegularTriangulation::Edge& e) const {
        auto fh = std::get<0>(e);
        auto i = std::get<1>(e);
        std::size_t h1 = std::hash<void*>()(&*fh); // hash Face_handle by address
        std::size_t h2 = std::hash<int>()(i);
        return h1 ^ (h2 << 1);
    }
};

struct Flip {
    std::array<VertexHandle, 4> flipVertices;
    int flipKind; 
    // 1: 1-3 flip
    // 2: 2-2 flip
    // 3: 3-1 flip
};

struct FlipHash {
    std::size_t operator()(const Flip& flip) const {
        // Hash all four vertex handles
        std::size_t h1 = std::hash<VertexHandle>()(flip.flipVertices[0]);
        std::size_t h2 = std::hash<VertexHandle>()(flip.flipVertices[1]);
        std::size_t h3 = std::hash<VertexHandle>()(flip.flipVertices[2]);
        std::size_t h4 = std::hash<VertexHandle>()(flip.flipVertices[3]);
        std::size_t h5 = std::hash<int>()(flip.flipKind);

        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3) ^ (h5 << 4);
    }
};

// equals operator for Flip
inline bool operator==(const Flip& f1, const Flip& f2) {
    return f1.flipVertices == f2.flipVertices && f1.flipKind == f2.flipKind;
}

// Maps
extern std::unordered_map<VertexHandle, int> vertexIndexMap;
extern std::unordered_map<std::array<VertexHandle, 3>, int, VertexHandleArrayHash> faceIndexMap;
extern std::unordered_map<Flip, int, FlipHash> flipIndexMap;


namespace TinyAD {
    inline int idx_from_handle(const VertexHandle& vh) {
        return vertexIndexMap.at(vh); // Retrieve the index from the map
    }
    
    inline int idx_from_handle(const std::array<VertexHandle, 3>& fh) {
        return faceIndexMap.at(fh); // Retrieve the index from the map
    }

    inline int idx_from_handle(const Flip& flip) {
        return flipIndexMap.at(flip); // Retrieve the index from the map
    }
}