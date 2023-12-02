// graph.h <Starter Code>
// Bill Munkh-Erdene
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Spring 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <algorithm>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
  typedef unordered_map<VertexT, WeightT> GraphMap;
  unordered_map<VertexT, GraphMap> AdjacencyList;
  set<VertexT> vertices;
  int numEdges;


 public:
  //
  // constructor:
  //
  // Constructs an empty graph where n is the max # of vertices
  // you expect the graph to contain.
  //
  //
  graph() {
    numEdges = 0;
  }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return vertices.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    return numEdges;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    if (vertices.count(v) == 1) {  // we're full:
      return false;
    }
    
    vertices.insert(v);
    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    if (vertices.count(from) == 0) {
      return false;
    }

    if (vertices.count(to) == 0) {
      return false;
    }

    if (AdjacencyList.count(from) == 0) {
      this->numEdges++;
    }
    else if ((AdjacencyList.at(from)).count(to) == 0) {
      this->numEdges++;
    }
    
    AdjacencyList[from][to] = weight;
    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    if (vertices.count(from) == 0) {
      return false;
    }

    if (vertices.count(to) == 0) {
      return false;
    }

    if (AdjacencyList.count(from) == 0) {
      return false;
    }
    else if ((AdjacencyList.at(from)).count(to) == 0) {
      return false;
    }
    
    GraphMap edgeW = AdjacencyList.at(from);
    weight = edgeW[to];
    
    return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;

    if (vertices.count(v) == 0) {
      return set<VertexT>();
    }

    if (AdjacencyList.count(v) != 0) {
      for (auto a : AdjacencyList.at(v)) {
        S.insert(a.first);
      }
    }
    
    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> verticesVector;
    
    for (const auto& a : vertices) {
      verticesVector.push_back(a);
    }

    return verticesVector;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;

    int i = 0;
    for(pair<VertexT, GraphMap> a : AdjacencyList) {
        i++;
        output << i << ". " << a.first << endl;
    }
      
    output << endl;
    output << "**Edges:" << endl;
    
    char c = 'A';
    for(pair<VertexT, GraphMap> a : AdjacencyList) {
        output << c << ": ";
        for(pair<VertexT, WeightT> b : AdjacencyList.at(a.first)) {
            output << "(" << a.first << ", " << b.first << ", " << b.second << "), ";
        }
        ++c;
        output << endl;
    }
    
    output << "**************************************************" << endl;
  }
};