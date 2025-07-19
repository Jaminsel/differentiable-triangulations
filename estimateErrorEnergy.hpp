#pragma once 
#include "types.hpp"
#include "globals.hpp"
#include <Eigen/Dense>
#include <iostream>

template <typename T>
T estimateErrorEnergy(Eigen::Vector2<T> p0, Eigen::Vector2<T> p1, Eigen::Vector2<T> p2, T c0, T c1, T c2, const Eigen::MatrixXd& imageMatrix){
    int pixelX_index = static_cast<int>(TinyAD::to_passive(p0.x()));
    // map vertex coordinates to image coordinates
    T p0X = p0.x()/triangulationSize * imageMatrix.cols(); // triangulation width
    T p0Y = p0.y()/triangulationSize * imageMatrix.rows(); // triangulation height
    T p1X = p1.x()/triangulationSize * imageMatrix.cols(); 
    T p1Y = p1.y()/triangulationSize * imageMatrix.rows(); 
    T p2X = p2.x()/triangulationSize * imageMatrix.cols(); 
    T p2Y = p2.y()/triangulationSize * imageMatrix.rows(); 

    // Calculate (pixel) Bounding Box
    int xMin = std::floor(std::min({TinyAD::to_passive(p0X), TinyAD::to_passive(p1X), TinyAD::to_passive(p2X)}));
    int xMax = std::floor(std::max({TinyAD::to_passive(p0X), TinyAD::to_passive(p1X), TinyAD::to_passive(p2X)}));
    int yMin = std::floor(std::min({TinyAD::to_passive(p0Y), TinyAD::to_passive(p1Y), TinyAD::to_passive(p2Y)}));
    int yMax = std::floor(std::max({TinyAD::to_passive(p0Y), TinyAD::to_passive(p1Y), TinyAD::to_passive(p2Y)}));
    if(xMax >= imageMatrix.cols()) xMax = imageMatrix.cols() - 1;
    if(yMax >= imageMatrix.rows()) yMax = imageMatrix.rows() - 1;

    // T faceArea = 0.0;
    T faceError = 0.0;
    T invDenominator = 1.0 / ((p1X - p0X)*(p2Y - p0Y) - (p2X - p0X)*(p1Y - p0Y));

    // iterate over all pixels in Bounding Box
    for(int pixelX = xMin; pixelX <= xMax; pixelX++){
        for(int pixelY = yMin; pixelY <= yMax; pixelY++){
            // compute pixel Color im image
            double pixelCenterX = pixelX + 0.5;
            double pixelCenterY = pixelY + 0.5;

            // calculate barycentric coordinates of pixel center with respect to triangle
            T a = ((p1X - pixelCenterX) * (p2Y - pixelCenterY) - (p2X - pixelCenterX) * (p1Y - pixelCenterY)) * invDenominator;
            T b = ((p2X - pixelCenterX) * (p0Y - pixelCenterY) - (p0X - pixelCenterX) * (p2Y - pixelCenterY)) * invDenominator;
            T c = 1 - a - b;

            // check if barycentric coordinates are non-negative
            if(a>=0 && b>=0 && c>=0){
                // calculate difference and add to FaceError 
                // faceArea += 1;
                T colorValue = a * c0 + b * c1 + c * c2;
                faceError += abs(colorValue - imageMatrix(pixelY, pixelX)); 
            }
        }
    }
    // T normalizedError;
    // if(faceArea == 0.0) normalizedError = 0.0;
    // else normalizedError = faceError / faceArea;
    return faceError;
    // return normalizedError;
}

double estimateErrorEnergyWDT(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, double c0, double c1, double c2, const Eigen::MatrixXd& imageMatrix){
    int pixelX_index = static_cast<int>(p0.x());
    // map vertex coordinates to image coordinates
    double p0X = p0.x()/triangulationSize * imageMatrix.cols(); // triangulation width
    double p0Y = p0.y()/triangulationSize * imageMatrix.rows(); // triangulation height
    double p1X = p1.x()/triangulationSize * imageMatrix.cols();
    double p1Y = p1.y()/triangulationSize * imageMatrix.rows(); 
    double p2X = p2.x()/triangulationSize * imageMatrix.cols(); 
    double p2Y = p2.y()/triangulationSize * imageMatrix.rows(); 

    // Calculate (pixel) Bounding Box
    int xMin = std::floor(std::min({p0X, p1X, p2X}));
    int xMax = std::floor(std::max({p0X, p1X, p2X}));
    int yMin = std::floor(std::min({p0Y, p1Y, p2Y}));
    int yMax = std::floor(std::max({p0Y, p1Y, p2Y}));
    if(xMax >= imageMatrix.cols()) xMax = imageMatrix.cols() - 1;
    if(yMax >= imageMatrix.rows()) yMax = imageMatrix.rows() - 1;

    // T faceArea = 0.0;
    double faceError = 0.0;
    double invDenominator = 1.0 / ((p1X - p0X)*(p2Y - p0Y) - (p2X - p0X)*(p1Y - p0Y));

    // iterate over all pixels in Bounding Box
    for(int pixelX = xMin; pixelX <= xMax; pixelX++){
        for(int pixelY = yMin; pixelY <= yMax; pixelY++){
            // compute pixel Color im image
            double pixelCenterX = pixelX + 0.5;
            double pixelCenterY = pixelY + 0.5;

            // calculate barycentric coordinates of pixel center with respect to triangle
            double a = ((p1X - pixelCenterX) * (p2Y - pixelCenterY) - (p2X - pixelCenterX) * (p1Y - pixelCenterY)) * invDenominator;
            double b = ((p2X - pixelCenterX) * (p0Y - pixelCenterY) - (p0X - pixelCenterX) * (p2Y - pixelCenterY)) * invDenominator;
            double c = 1 - a - b;

            // check if barycentric coordinates are non-negative
            if(a>=0 && b>=0 && c>=0){
                // calculate difference and add to FaceError 
                // faceArea += 1;
                double colorValue = a * c0 + b * c1 + c * c2;
                faceError += abs(colorValue - imageMatrix(pixelY, pixelX)); 
            }
        }
    }
    // T normalizedError;
    // if(faceArea == 0.0) normalizedError = 0.0;
    // else normalizedError = faceError / faceArea;
    return faceError;
    // return normalizedError;
}