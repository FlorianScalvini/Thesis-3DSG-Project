//
// Created by ubuntu on 14/09/22.
//

#ifndef OUTDOORNAV_NODE_H
#define OUTDOORNAV_NODE_H

#include <string>
#include <utility>
#include <vector>
#include "Common/geographic_coordinate.h"
#include <map>

struct Way;
struct Node;
struct Edge;

struct Node
{
    GeographicCoordinate coordinate;
    const std::string id;
    int numberWay;
    std::vector<Edge*> lst_edges;
    bool crossing;

    Node(const std::string id, float longitude, float latitude, bool crossing=false) : id(id)
    {
        this->coordinate = GeographicCoordinate(longitude, latitude);
        this->numberWay = 0;
        this->crossing = crossing;
    }
};

struct Edge {
    std::string id_edge;
    Node* node_A;
    Node* node_B;
    double weight;
    Way* parentWay;
    Edge(std::string id_edge, Node* node_A, Node* node_B, float weight, Way* parentWay)
    {
        this->id_edge = id_edge;
        this->node_A = node_A;
        this->node_B = node_B;
        this->weight = weight;
        this->parentWay = parentWay;
    };
};

struct Way
{
    std::string id;
    std::map<std::string, std::string> attribut;
    std::vector<Node*> lst_nodes;
    std::vector<Edge*> lst_edges;
    Way(std::string id) : id(id) {};
};



#endif //OUTDOORNAV_NODE_H