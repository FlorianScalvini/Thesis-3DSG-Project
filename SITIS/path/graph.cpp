//
//  Graph.cpp
//  ddd
//
//  Created by Florian Scalvini on 19/06/2022.
//

#include "graph.h"
#include <iostream>
#include <fstream>
#include <queue>

Graph::Graph(std::vector<int> listId)
{
    this->numNode = listId.size();
    for(int i = 0; i < numNode; i++)
    {
        this->nodes.insert(std::pair<int, Node*>(listId[0], new Node(listId[0])));
    }
}

Graph::Graph(const char * path)
{
    std::ifstream infile(path);
    std::string line;
    std::vector<std::pair<unsigned int, std::vector<int>>> links;
    std::vector<std::pair<unsigned int, std::vector<int>>> classes;
    std::vector<Node> graph;
    if (!infile.is_open()) {
        std::cerr << "Could not open the file - '" << path << "'" << std::endl;
        return;
    }
    int id;

    while(std::getline(infile, line))
    {
        char * lastChr = &line[line.size() -1];
        char * ptrLine = &line[0];
        id = extractValue(&ptrLine, lastChr);
        this->nodes.insert(std::pair<int, Node*>(id, new Node(id)));
        ptrLine++;
        links.emplace_back(id, extractListValue(&ptrLine, ',', lastChr));
        ptrLine++;
        classes.emplace_back(id, extractListValue(&ptrLine, ',', lastChr));
    }
    this->numNode = links.size();
    for(unsigned int i = 0; i < numNode; i++)
    {
        for(unsigned int j=0; j < links[i].second.size(); j++)
        {
            addLink(links[i].first, links[i].second[j], classes[i].second[j]);
        }
    }
}

Graph::~Graph()
{
}

Node* Graph::getNode(unsigned int indice)
{
    if(this->nodes.find(indice) != this->nodes.end())
        return nodes[indice];
    else
        return nullptr;
}

int Graph::getClasse(unsigned int src, unsigned int dst)
{
    if(this->nodes.find(src) == this->nodes.end())
        return 0;
    if(this->nodes[src]->classes.find(dst) == this->nodes[src]->classes.end())
        return 0;
    return nodes[src]->classes[dst];
}

void Graph::addLink(unsigned int src, unsigned int dst, unsigned int classe)
{

    Node* srcNode = this->getNode(src);
    Node* dstNode = this->getNode(dst);
    if(srcNode != nullptr && dstNode != nullptr )
    {
        this->nodes[src]->links.insert(dstNode);
        this->nodes[src]->classes.insert({dst, classe});
    }
    else
    {
        std::cerr<<"Impossible to build the connexion between the Node "<< src << " and" << dst << ".";
    }
}

void Graph::showGraph()
{
    for(auto node : this->nodes)
    {
        std::cout<<node.first << ":";
        for (auto neighbors : node.second->links)
        {
            std::cout<<" "<<neighbors->label;
        }
        std::cout<<"."<<std::endl;
    }
}


std::vector<int> Graph::extractListValue( char ** txt,  char separator, const char * lastTxt)
{
    std::vector<int> result;
    while(**txt != separator)
    {
        result.push_back(extractValue(txt, lastTxt));
    }
    return result;
}

void Graph::skipSpace( char ** txt, const char * lastTxt)
{
    while(txt != &lastTxt && **txt == ' ')
        *txt +=1;
}

int Graph::extractValue(char ** txt, const char * lastTxt) {
    std::string numberString;
    int number;
    skipSpace(txt, lastTxt);
    while (txt != &lastTxt && **txt >= 48 && **txt <= 57)
    {
        numberString += **txt;
        *txt += 1;
    }
    skipSpace(txt, lastTxt);
    number = std::stoi(numberString);
    return number;
}