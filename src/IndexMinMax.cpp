#include "../include/IndexMinMax.h"
#include <random>

Eigen::VectorXd findAllMinIndices(const Eigen::VectorXd &eigenVec) {
    // Convert Eigen vector to std::vector for easier processing with STL algorithms
    std::vector<double> values(eigenVec.data(), eigenVec.data() + eigenVec.size());

    std::vector<int> minIndices;
    
    // Step 1: Find the minimum value
    double minVal = *std::min_element(values.begin(), values.end());

    // Step 2: Collect all indices where the value matches minVal
    for (int i = 0; i < values.size(); ++i) {
        if (values[i] == minVal) {
            minIndices.push_back(i);
        }
    }
    
    // Step 3: Convert std::vector of indices to Eigen::VectorXd
    Eigen::VectorXd minIndexes = Eigen::Map<Eigen::VectorXi>(minIndices.data(), minIndices.size()).cast<double>();
    return minIndexes;
}

Eigen::VectorXd findAllMaxIndices(const Eigen::VectorXd &eigenVec) {
    // Convert Eigen vector to std::vector for easier processing with STL algorithms
    std::vector<double> values(eigenVec.data(), eigenVec.data() + eigenVec.size());

    std::vector<int> maxIndices;
    
    // Step 1: Find the minimum value
    double maxVal = *std::max_element(values.begin(), values.end());

    // Step 2: Collect all indices where the value matches minVal
    for (int i = 0; i < values.size(); ++i) {
        if (values[i] == maxVal) {
            maxIndices.push_back(i);
        }
    }
    
    // Step 3: Convert std::vector of indices to Eigen::VectorXd
    Eigen::VectorXd minIndexes = Eigen::Map<Eigen::VectorXi>(maxIndices.data(), maxIndices.size()).cast<double>();
    return minIndexes;
}