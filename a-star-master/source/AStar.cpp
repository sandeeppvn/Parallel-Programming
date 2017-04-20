#include "AStar.hpp"
#include <algorithm>
#include <time.h>
#include <iostream>
#include <omp.h>
clock_t start, end, start_b,end_b;
double cpu_time_used1,cpu_insert,cpu_conact,cpu_det_in,cpu_det_out,cpu_heur,cpu_col;

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = { 
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.insert(new Node(source_));
    start_b = clock();
    int count=0;
    while (!openSet.empty()) {
        count++;
        current = *openSet.begin();
        for (auto node : openSet) {
            if (node->getScore() <= current->getScore()) {
                current = node;
            }
        }
        
        if (current->coordinates == target_) {
            break;
        }
        start = clock();
        closedSet.insert(current);
        
        
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));
        end = clock();
        cpu_insert += ((double) (end - start)) / CLOCKS_PER_SEC;

        for (uint i = 0; i < directions; ++i) {
            
            Vec2i newCoordinates(current->coordinates + direction[i]);
            start_b=clock();
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }
            end_b=clock();
            cpu_col += ((double) (end_b - start_b)) / CLOCKS_PER_SEC;
            
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                cpu_heur += ((double) (end - start)) / CLOCKS_PER_SEC;
                openSet.insert(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
            
        }
    }
    end_b=clock();
    cpu_time_used1 += ((double) (end_b - start_b)) / CLOCKS_PER_SEC;

    start=clock();
    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }
    end= clock();
    cpu_conact += ((double) (end - start)) / CLOCKS_PER_SEC;
    releaseNodes(openSet);
    releaseNodes(closedSet);
    // std::cout << cpu_insert << "cpu-insert" << "\n";
    // std::cout << cpu_time_used1 << "cpu_time_used1" << "\n";
    // std::cout << cpu_conact << "cpu_conact" << "\n";
    
    // std::cout << cpu_det_out << "cpu_det_out" << "\n";
    // std::cout << cpu_heur << "cpu_heur" << "\n";
    // std::cout << cpu_col << "cpu_col" << "\n";
    // std::cout << count;
    
    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    start=clock();
    int flag=0;
    AStar::Node* a;
    // AStar::Node *current = nullptr;
    /*auto current = nodes_.begin();
    while(current!=nodes_.end())
    {
        // current = *nodes_.begin();
        for (auto node : nodes_) {
            if (node->getScore() == current->getScore()) {
                current = node;
            }
        }
        
        if (current->coordinates == coordinates_) {
            a=current;
            flag=1;
        }
        current++;
    }*/
    
    /*auto first= nodes_.begin();
    auto last= nodes_.end();
    #pragma omp parallel
    while(first!=last){
        if((*first)->coordinates == coordinates_){
            a=*first;flag=1;
        }
        ++first;
    }*/


    /*for (auto node : nodes_) {

        if (node->coordinates == coordinates_) {
            a=node;
            // std::cout << "" << "\n\n";
            flag=1;
        }
    }*/

    auto it=nodes_.begin();
    // #pragma omp parallel
    while(it!=nodes_.end() && !flag){
        if((*it)->coordinates==coordinates_){
                        a=*it;
                        flag=1;
                    }
                    it++;
                }


        // auto it=nodes_.begin();
        // auto it1=it,it2=it;
        // while(it!=nodes_.end() && !flag){
        //     it1=it;
        //     it2=it;
        //     #pragma omp sections
        //     {
        //         #pragma omp section
        //         {
        //             std::cout << coordinates_.x << " " << coordinates_.y << "\n";
        //             if((*it1)->coordinates==coordinates_){
        //                 a=*it1;
        //                 flag=1;
        //             }
        //             it++;
        //         }
        //         #pragma omp section
        //         {
        //             if((*it)->coordinates==coordinates_){
        //                 a=*it;
        //                 flag=1;
        //             }
        //             it=*(&it+1);
        //         }
                
                
        //     }
        // }


    if(flag==1)
        return a;
    end=clock();
    cpu_det_out += ((double) (end - start)) / CLOCKS_PER_SEC;
    // std::cout << cpu_det_out << "cpu_det_out" << "\n";
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    start=clock();
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        end=clock();
        cpu_det_in += ((double) (end - start)) / CLOCKS_PER_SEC;
        // std::cout << cpu_det_in << "cpu_det_in" << "\n";
        return true;
    }
    end=clock();
    cpu_det_in += ((double) (end - start)) / CLOCKS_PER_SEC;
    // std::cout << cpu_det_in << "cpu_det_in" << "\n";
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}