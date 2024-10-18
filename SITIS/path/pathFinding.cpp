//
// Created by ubuntu on 20/06/22.
//

#include "pathFinding.h"
#include <queue>
#include <algorithm>
#include <iostream>

PathFinding::PathFinding(Graph* graph)
{
    this->graph = graph;
    this->currPosition = 0;
}


void PathFinding::computeBFS(unsigned int src, unsigned int dst)
{
    markerList.clear();
    std::set<unsigned int> reversePath;
    std::map<unsigned int, unsigned int> activateLink;
    std::queue<Node*> queue;
    bool foundDst = false;

    queue.push(graph->getNode(src));
    while(!queue.empty())
    {
        Node* curr = queue.front();
        queue.pop();
        if(curr->label == dst)
        {
            foundDst = true;
            break;
        }
        for(auto node : curr->links)
        {
            if(activateLink.find(node->label) == activateLink.end())
            {
                activateLink[node->label] = curr->label;
                queue.push(node);
            }
        }
    }
    if(foundDst) {
        unsigned int indicePath = dst;
        markerList.push_back(dst);
        while (indicePath != src) {
            indicePath = activateLink[indicePath];
            markerList.push_back(indicePath);
        }
        std::reverse(markerList.begin(), markerList.end());
        this->currPosition = 0;
    }
}

Node* PathFinding::getCurrentNode()
{
    if(this->currPosition >= 0 && this->currPosition < markerList.size())
        return  this->graph->getNode(markerList[this->currPosition]);
    else
        return nullptr;
}

bool PathFinding::setCurrentNode(unsigned int src)
{
    if(graph->getNode(src) != nullptr)
    {
        currPosition = src;
        return true;
    }
    else
    {
        std::cout<<"This node is unknown." << std::endl;
        return false;
    }

}


unsigned int PathFinding::newPath(unsigned int dst)
{
    computeBFS(currPosition, dst);
    if(this->markerList.size() >= 0)
    {
        return this->markerList.size();
    }
    else
        return 0;
}

bool PathFinding::changeToClosestNode(unsigned int label)
{
    bool result = false;
    unsigned int i = currPosition;
    while(i <= markerList.size())
    {
        if(label == markerList[i])
        {
            result = true;
            currPosition = i;
            break;
        }
        i++;
    }
    return result;
}


Node* PathFinding::update()
{
    if(currPosition < markerList.size()-1)
    {
        this->currPosition++;
    }
    return this->getCurrentNode();
}

void PathFinding::showPath()
{
    for(unsigned int i = currPosition; i < markerList.size(); i++)
    {
        std::cout<<markerList[i];
        if(i < (markerList.size()-1))
            std::cout<<" --> ";
        else
            std::cout<<std::endl;
    }
}
int PathFinding::getNextNode() {
    if(currPosition < markerList.size()-1)
    {
        return markerList[currPosition + 1];
    }
    else
        return -1;
}