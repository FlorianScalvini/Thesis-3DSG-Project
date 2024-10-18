//
// Created by ubuntu on 14/09/22.
//

#ifndef OUTDOORNAV_MAP_H
#define OUTDOORNAV_MAP_H
#include <string>
#include "Node.h"
#include "Common/geographic_coordinate.h"
#define Path_UCS 0
#define Path_BFS 1



class Map {
public:
    explicit Map(const std::string& path_osm);
    Node* getNearestPoint(GeographicCoordinate point);
    static std::vector<Node*> searchPath(Node* node_A, Node* node_B, int algorithm=Path_UCS);
    Node* getNodeFromId(const std::string& id);
    Way* getWayFromId(const std::string& id);
    Edge* getEdgeFromExt(const std::string& id_edge);

private:
    bool inBound(GeographicCoordinate point) const; // Check if the coordinate is the map's area
    void addNode(std::string id, float longitude, float latitude, bool crossing=false); // Add new node to the graph
    void parseXML(const std::string& path_osm); // Parse the XML file (or OSM file)
    static std::vector<Node*> compute_UCS(Node* node_A, Node* node_B); // Compute the Uniform Cost Search pathfinding algorithm
    static std::vector<Node*> compute_AStar(Node* node_A, Node* node_B); // Compute the A* pathfinding algorithm

    std::vector<std::vector<std::vector<Node*>>> gridNodes; // Grid of node's list

    std::map<std::string , Node> nodes; // Map of node : Key : String of the id / Value : Node type
    std::map<std::string, Way> way_data; // Map of Way : Key : String of the id / Value : Way type
    std::map<std::string, Edge> edge_data; // Map of Edge : Key : String of the edge / Value : Edge type
    float minLat, maxLat, minLong, maxLong, incLong, incLat; // Navigation area Min/Max
    unsigned int nbGridLong, nbGridLat; // Number of subdivision of the navigation area

    struct NodePathFinding
    {
        Node* node;
        int indexParent;
        float cost;
        explicit NodePathFinding(Node* node, float cost = 0, int indexParent = -1)
        {
            this->indexParent = indexParent;
            this->node = node;
            this->cost = cost;
        }
    };

};


#endif //OUTDOORNAV_MAP_H
