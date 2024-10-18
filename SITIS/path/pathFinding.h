//
// Created by ubuntu on 20/06/22.
//

#ifndef STAG_PATHFINDING_H
#define STAG_PATHFINDING_H

#include "graph.h"


class PathFinding {
public:
    explicit PathFinding(Graph* graph);
    unsigned int newPath(unsigned int dst);
    Node* update();
    Node* getCurrentNode();
    int getNextNode();
    bool setCurrentNode(unsigned int src);
    bool changeToClosestNode(unsigned int label);
    void showPath();

private:
    std::vector<unsigned int> markerList;
    unsigned int currPosition;
    void computeBFS(unsigned int src, unsigned int dst);
    Graph *graph;
};


#endif //STAG_PATHFINDING_H
