//
//  Graph.hpp
//  ddd
//
//  Created by Florian Scalvini on 19/06/2022.
//

#ifndef Graph_h
#define Graph_h

#include <stdio.h>
#include <map>
#include <set>
#include <vector>
#include <utility>


struct Node
{

    unsigned int label;
    std::set<Node*> links;
    std::map<unsigned int, unsigned int> classes;
    explicit Node(unsigned int label)
    {
        this->label = label;
    }
};

class Graph {
public:
    explicit Graph(std::vector<int> listId);
    explicit Graph(const char * path);
    ~Graph();
    Node* getNode(unsigned int indice);
    void addLink(unsigned int src, unsigned int dst, unsigned int classe = 0);
    int getClasse(unsigned int src, unsigned int dst);
    void showGraph();

private:
    unsigned int numNode;
    std::map<unsigned int, Node*> nodes;
    static void skipSpace( char ** txt, const char * lastTxt);
    static int extractValue( char ** txt, const char * lastTxt);
    static std::vector<int> extractListValue( char ** txt,  char separator, const char * lastTxt);
};

#endif /* Graph_h */
