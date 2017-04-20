
#include <iostream>
#include "source/AStar.hpp"

int main()
{
    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({200, 200});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    
    std::cout << "Generate path ... \n";
    // This method returns vector of coordinates from target to source.
    auto path = generator.findPath({0, 0}, {125, 180});
    
    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
}