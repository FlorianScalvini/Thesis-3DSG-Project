//
// Created by ubuntu on 14/09/22.
//


#include "RapidXML/rapidxml.hpp"
#include "Map.h"
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <istream>
#include <cstring>
#include <map>
#include <utility>
#include <queue>
#include <algorithm>
#include <cmath>

#define BoundXMin 4.91
#define BoundXMax 5.09
#define BoundYMin 42.27
#define BoundYMax 49.36
#define incBoundX 0.05
#define incBoundY 0.025




Map::Map(const std::string& path_osm) :
minLong(BoundXMin), maxLong(BoundXMax), maxLat(BoundYMax), minLat(BoundYMin), incLong(incBoundX), incLat(incBoundY){
    this->nbGridLong = (const unsigned int) ceil((double)(this->maxLong - this->minLong) / this->incLong);
    this->nbGridLat = (const unsigned int) ceil((double)(this->maxLat - this->minLat) / this->incLat);
    this->gridNodes.clear();
    for(int i = 0; i < this->nbGridLong; i++)
    {
        this->gridNodes.emplace_back();
        for(int j = 0; j < this->nbGridLat; j++)
            this->gridNodes[i].emplace_back();
    }
    parseXML(path_osm);
    /* Add Edge to way */
    std::map<std::string, Way>::iterator it_way;
    Node* node_A;
    float distance;

    // Penality score depending of the street attributs
    std::map<std::string, float> path_penalty = {
            {"proposed", 100.0},
            {"rest_area", 100.0},
            {"motorway_link", 100.0},
            {"motorway", 100.0},
            {"trunk_link", 100.0},
            {"construction", 100.0},
            {"primary", 10.0},
            {"unclassified", 6.0},
            {"road", 6.0},
            {"secondary", 4.0},
            {"tertiary", 3.0},
            {"service", 2.0},
            {"footway", 1.0},
            {"cycleway", 1.8},
            {"path", 1.0},
            {"crossing", 10.0},
            {"residential", 2.0},
            {"living_street", 1.8},
            {"track", 1.2},
            {"steps", 1.2},
            {"pedestrian", 1.0},
            {"secondary_link", 4.0},
            {"primary_link", 10.0},
            {"tertiary_link", 3.0},
            {"bridleway", 1.2},
            {"trunk", 100.0},
            {"raceway", 100.0},
            {"corridor", 50.0},
            {"services", 2.0},
            {"busway", 10.0},
            {"bus_stop", 10.0},
            {"bus_guideway", 2.0}
    };

    for (it_way = way_data.begin(); it_way != way_data.end(); it_way++)
    {
        int count = 0;
        float penalty = 100.0; // Max penalty
        if(it_way->second.attribut.find("highway") != it_way->second.attribut.end() && path_penalty.find(it_way->second.attribut["highway"]) != path_penalty.end())
                penalty = path_penalty[it_way->second.attribut["highway"]];

        for(int i = 0; i < it_way->second.lst_nodes.size(); i++){
            if(i == 0)
                node_A = it_way->second.lst_nodes[0];
            else {
                std::string edge_id = it_way->first + "_" + std::to_string(count);
                count++;
                distance = GeographicCoordinate::toDistance(node_A->coordinate, it_way->second.lst_nodes[i]->coordinate);
                edge_data.insert(std::pair<std::string, Edge>(edge_id, {edge_id, node_A, it_way->second.lst_nodes[i], distance * penalty, &it_way->second}));
                Edge *edge = &edge_data.at(edge_id);
                node_A->lst_edges.push_back(edge);
                node_A = it_way->second.lst_nodes[i];
                node_A->lst_edges.push_back(edge);
            }
        }
    }
}

void Map::parseXML(const std::string& path_osm) {
    rapidxml::xml_document<> doc;
    std::ifstream file(path_osm);
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    std::string content(buffer.str());
    doc.parse<0>(&content[0]);

    rapidxml::xml_node<>* root = doc.first_node();
    rapidxml::xml_node<>* itemTmp = root->first_node();
    // With the xml example above this is the <document/> node
    std::string itemName = root->name();

    auto* bound = new float[4];
    rapidxml::xml_attribute<>* attr;
    for (;nullptr!=itemTmp;itemTmp = itemTmp->next_sibling())
    {
        if(strcmp(itemTmp->name(), "bounds") == 0)
        {
            attr = itemTmp->first_attribute();
            int i = 0;
            for(;nullptr!=attr;attr = attr->next_attribute(), i++)
            {
                bound[i] = std::stof(attr->value());
            }
        }
        else if(strcmp(itemTmp->name(), "node") == 0)
        {

            float longitude = std::atof(itemTmp->first_attribute("lon")->value());
            float latitude = std::atof(itemTmp->first_attribute("lat")->value());
            bool crossing = false;
            const char* id = itemTmp->first_attribute("id")->value();
            rapidxml::xml_node<>* itemChildWay = itemTmp->first_node();

            for (;nullptr!=itemChildWay;itemChildWay = itemChildWay->next_sibling())
                if(strcmp(itemChildWay->name(), "tag") == 0 && strcmp(itemChildWay->first_attribute("k")->value(), "highway") == 0 && strcmp(itemChildWay->first_attribute("v")->value(), "crossing") == 0)
                    crossing = true;
            this->addNode(id, longitude, latitude, crossing);
            //node_data.insert(std::pair<std::string, Node>(id, {id, longitude, latitude}));
        }
        else if(strcmp(itemTmp->name(), "way") == 0)
        {
            std::string id = itemTmp->first_attribute("id")->value();
            Way way = Way(id);
            rapidxml::xml_node<>* itemChildWay = itemTmp->first_node();
            for (;nullptr!=itemChildWay;itemChildWay = itemChildWay->next_sibling()) {
                if(strcmp(itemChildWay->name(), "nd") == 0)
                {
                    std::string node_id = itemChildWay->first_attribute("ref")->value();
                    Node* node = this->getNodeFromId(node_id);
                    if(node == nullptr)
                        continue;
                    node->numberWay++;
                    way.lst_nodes.push_back(node);
                }
                else if(strcmp(itemChildWay->name(), "tag") == 0)
                {
                    way.attribut.insert(std::pair<std::string, std::string>(itemChildWay->first_attribute("k")->value(), itemChildWay->first_attribute("v")->value()));
                }
            }
            way_data.insert({id, way});
        }
    }
}

void Map::addNode(std::string id, float longitude, float latitude, bool crossing) {
    Node node = Node(id, longitude, latitude, crossing);
    if (this->inBound(GeographicCoordinate(longitude, latitude)))
    {
        int x_grid = int(floor((longitude - this->minLong) / this->incLong));
        int y_grid = int(floor((latitude - this->minLat) / this->incLat));
        this->nodes.insert({id, node});
        this->gridNodes[x_grid][y_grid].push_back(&this->nodes.at(id));
    }
}

std::vector<Node*> Map::searchPath(Node *node_A, Node* node_B, int algorithm) {
    if(algorithm == Path_UCS)
        return Map::compute_UCS(node_A, node_B);
    else
        return Map::compute_AStar(node_A, node_B); // Modify
}


Way *Map::getWayFromId(const std::string& id) {
    auto item = way_data.find(id);
    if (item != this->way_data.end())
        return &item->second;
    return nullptr;
}

Node *Map::getNodeFromId(const std::string& id) {
    auto item = this->nodes.find(id);
    if (item != this->nodes.end())
        return &item->second;
    return nullptr;
}

Node *Map::getNearestPoint(GeographicCoordinate point) {
    Node* nearestNode;
    if (this->inBound(point))
    {
        int x_grid = int(floor((point.longitude - this->minLong) / this->incLong));
        int y_grid = int(floor((point.latitude - this->minLat) / this->incLat));
        nearestNode = this->gridNodes[x_grid][y_grid][0];
        double distance = GeographicCoordinate::toDistance(point, nearestNode->coordinate);

        for (auto it = this->gridNodes[x_grid][y_grid].begin() + 1; it != this->gridNodes[x_grid][y_grid].end(); ++it) {
            Node* currentNode = *it;
            float new_distance = GeographicCoordinate::toDistance(point, currentNode->coordinate);
            if (distance > new_distance){
                distance = new_distance;
                nearestNode = currentNode;
            }
        }
        return nearestNode;
    }
    return nullptr;
}

Edge *Map::getEdgeFromExt(const std::string& id_edge) {
    auto item = edge_data.find(id_edge);
    if (item != this->edge_data.end())
        return &item->second;
    return nullptr;
}

bool Map::inBound(GeographicCoordinate point) const {
    return (this->minLong <= point.longitude && point.longitude <= this->maxLong && this->minLat <= point.latitude && point.latitude <= this->maxLat);
}


/*
 * A Star algorithm
 */
std::vector<Node *> Map::compute_AStar(Node *node_A, Node *node_B) {

    std::vector<NodePathFinding> nodes;
    std::vector<int> queue;
    std::vector<int> visitedNodes;
    nodes.emplace_back(node_A, 0, -1);
    queue.emplace_back(0);
    std::vector<Node*> result;
    while(!queue.empty()) {
        int idx_current_queue = 0;
        double h_A = GeographicCoordinate::toDistance(nodes[queue[idx_current_queue]].node->coordinate, node_B->coordinate);
        double h_B = 0;
        for (int i = 1; i < queue.size(); i++) {
            h_B = GeographicCoordinate::toDistance(nodes[queue[i]].node->coordinate, node_B->coordinate);
            if (nodes[queue[idx_current_queue]].cost + h_A> nodes[queue[i]].cost + h_B) {
                idx_current_queue = i;
                h_A = h_B;
            }
        }

        NodePathFinding currentNode = nodes[queue[idx_current_queue]];
        if (currentNode.node->id.c_str() == node_B->id.c_str()) {
            int index = idx_current_queue;
            while (index != -1) {
                result.emplace_back(nodes[index].node);
                index = nodes[index].indexParent;
            }
            break;
        }

        for(int i = 0; i < currentNode.node->lst_edges.size(); i++) {
            Node *node_edge_ext;
            if (currentNode.node->lst_edges[i]->node_A == currentNode.node)
                node_edge_ext = currentNode.node->lst_edges[i]->node_B;
            else
                node_edge_ext = currentNode.node->lst_edges[i]->node_A;

            float current_edge_cost = currentNode.node->lst_edges[i]->weight + currentNode.cost;
            bool isVisited = false;
            int idx_visited_while = 0;
            while (!isVisited && idx_visited_while < visitedNodes.size()) {
                if (nodes[visitedNodes[idx_visited_while]].node == node_edge_ext) {
                    if (nodes[visitedNodes[isVisited]].cost > current_edge_cost) {
                        nodes[visitedNodes[isVisited]].indexParent = queue[idx_current_queue];
                        nodes[visitedNodes[isVisited]].cost = current_edge_cost;
                    }
                    isVisited = true;
                }
                idx_visited_while++;
            }

            if (!isVisited) // Si le noeud a été visité
            {
                int idx = 0;
                bool isInQueue = false;
                while (!isInQueue && idx < queue.size()) {
                    if (nodes[queue[idx]].node == node_edge_ext) {
                        isInQueue = true;
                        if (current_edge_cost < nodes[queue[idx]].cost) {
                            nodes[queue[idx]].indexParent = queue[idx_current_queue];
                            nodes[queue[idx]].cost = current_edge_cost;
                        }
                    }
                    idx++;
                }
                if (!isInQueue) {
                    nodes.emplace_back(node_edge_ext, current_edge_cost, queue[idx_current_queue]);
                    queue.emplace_back(nodes.size() - 1);
                }
            }

        }
        visitedNodes.emplace_back(queue[idx_current_queue]);
        queue.erase(queue.begin() + idx_current_queue);
    }
    std::reverse(result.begin(), result.end());
    return result;
}

/*
 * Uniform Cost Search algorithm
 */
std::vector<Node *> Map::compute_UCS(Node *node_A, Node *node_B) {
    std::vector<NodePathFinding> nodes;
    std::vector<int> queue;
    std::vector<int> visitedNodes;
    nodes.emplace_back(node_A, 0, -1);
    queue.emplace_back(0);
    float best_cost = -1;
    while (!queue.empty()) {
        int idx_current_queue = 0;
        for (int i = 1; i < queue.size(); i++) {
            if (nodes[queue[idx_current_queue]].cost > nodes[queue[i]].cost) {
                idx_current_queue = i;
            }
        }

        NodePathFinding currentNode = nodes[queue[idx_current_queue]];
        for (int i = 0; i < currentNode.node->lst_edges.size(); i++) {
            Node *node_edge_ext;
            if (currentNode.node->lst_edges[i]->node_A == currentNode.node)
                node_edge_ext = currentNode.node->lst_edges[i]->node_B;
            else
                node_edge_ext = currentNode.node->lst_edges[i]->node_A;

            float current_edge_cost = currentNode.node->lst_edges[i]->weight + currentNode.cost;
            if (strcmp(node_edge_ext->id.c_str(), node_B->id.c_str()) == 0) {
                best_cost = current_edge_cost;
            }

            bool isVisited = false;
            int idx_visited_while = 0;
            while (!isVisited && idx_visited_while < visitedNodes.size()) {
                if (nodes[visitedNodes[idx_visited_while]].node == node_edge_ext) {
                    if (nodes[visitedNodes[isVisited]].cost > current_edge_cost) {
                        nodes[visitedNodes[isVisited]].indexParent = queue[idx_current_queue];
                        nodes[visitedNodes[isVisited]].cost = current_edge_cost;
                    }
                    isVisited = true;
                }
                idx_visited_while++;
            }

            if (!isVisited) // Si le noeud a été visité
            {
                int idx = 0;
                bool isInQueue = false;
                while (!isInQueue && idx < queue.size()) {
                    if (nodes[queue[idx]].node == node_edge_ext) {
                        isInQueue = true;
                        if (current_edge_cost < nodes[queue[idx]].cost) {
                            nodes[queue[idx]].indexParent = queue[idx_current_queue];
                            nodes[queue[idx]].cost = current_edge_cost;
                        }
                    }
                    idx++;
                }
                if (!isInQueue) {
                    nodes.emplace_back(node_edge_ext, current_edge_cost, queue[idx_current_queue]);
                    queue.emplace_back(nodes.size() - 1);
                }
            }

        }
        visitedNodes.emplace_back(queue[idx_current_queue]);
        queue.erase(queue.begin() + idx_current_queue);
        if (best_cost != -1) {
            for (int idx_queue = (int) queue.size() - 1; idx_queue >= 0; idx_queue--) {
                if (nodes[queue[idx_queue]].cost > best_cost)
                    queue.erase(queue.begin() + idx_queue);
            }
        }
    }

    std::vector<Node *> result;
    int isReach = -1;
    for (int i = 0; i < visitedNodes.size(); i++) {
        if (nodes[visitedNodes[i]].node == node_B) {
            isReach = i;
            break;
        }
    }
    if (isReach != -1) {
        unsigned int index = visitedNodes[isReach];
        while (index != -1) {
            result.emplace_back(nodes[index].node);
            index = nodes[index].indexParent;
        }
        std::reverse(result.begin(), result.end());
    }
    return result;
}
