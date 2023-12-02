// application.cpp
// Bill Munkh-Erdene
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
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <limits>
#include <queue>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"

using namespace std;
using namespace tinyxml2;
const double INF = numeric_limits<double>::max();

class prioritize
{
public:
  bool operator()(const pair<int,int>& p1, const pair<int,int>& p2) const
  {
    return p1.second > p2.second; 
  }
};

bool searchBuilding(vector<BuildingInfo> Buildings, BuildingInfo& pBuilding, string input) {
  for (auto b: Buildings) {
    if (b.Abbrev == input) {
      pBuilding = b;
      return true;
    }
  }
  for (auto b : Buildings) {
    if (b.Fullname.find(input) != string::npos) {
      pBuilding = b;
      return true;
    }
  }
  return false;
}

long long nearestNode(vector<FootwayInfo>& Footways, map<long long, Coordinates>& Nodes, BuildingInfo building) {
  double min = INF;
  double distance = 0.0;
  long long nNode = 0;
  long long nBuilding = 0;

  for (auto a : Footways) {
    
    for (size_t i = 0; i < (int)a.Nodes.size(); i++) {
      nNode = a.Nodes.at(i);
      distance = distBetween2Points(building.Coords.Lat, building.Coords.Lon, Nodes.at(nNode).Lat, Nodes.at(nNode).Lon);

      if (distance < min) {
        min = distance;
        nBuilding = nNode;
        
      }
    }
  }
  return nBuilding;
}

void DijkstraShortestPath(long long startV, graph<long long, double> G, map <long long, Coordinates>& Nodes, map<long long, long long>& predV, map<long long, double>& pathV) {
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQueue;
  set<long long> visited;
  long long shortest1Path;
  double shortest2Path;
  double vertexW;
  double distance;

  for (auto n : Nodes) {
    pathV[n.first] = INF;
    predV[n.first] = 0;
    unvisitedQueue.push(make_pair(n.first, INF));
  }
  
  pathV[startV] = 0;
  unvisitedQueue.push(make_pair(startV, 0));

  while (unvisitedQueue.empty() == false) {
    shortest1Path = unvisitedQueue.top().first;
    shortest2Path = unvisitedQueue.top().second;
    unvisitedQueue.pop();

    if(pathV[shortest1Path] == INF) {
      break;
    } else if (visited.count(shortest1Path)) {
      continue;
    }
    
    visited.insert(shortest1Path);
    
    for (auto g : G.neighbors(shortest1Path)) {
      G.getWeight(shortest1Path, g, vertexW);
      distance = shortest2Path + vertexW;

      if (distance < pathV[g]) {
        pathV[g] = distance;
        predV[g] = shortest1Path; 
        unvisitedQueue.push(make_pair(g, distance));
      
      }
    }
  }
}

vector<long long> getPath(map<long long, long long> predV, long long endV) {
  vector<long long> path;
  long long currV;

  currV = predV.at(endV);
  path.push_back(endV);

  while(predV.at(endV) != 0 && predV.at(currV) != 0) {
    path.push_back(currV);
    currV = predV.at(currV);
  }
  path.push_back(currV);

  if (predV.at(endV) == 0) {
    path.clear();
    path.push_back(endV);
    return path;
  }
  return path;
}

void printPath(vector <long long> path) {
  cout << "Path: " << path.at(0);
  for (size_t i = 1; i < (int)path.size(); i++) {
    cout << "->" << path.at(i);
  }
  cout << endl;
}
//
// Implement your standard application here
//
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double> G) {
  string person1Building, person2Building;
  BuildingInfo input1Building, input2Building;
  BuildingInfo centerBuilding;
  bool found = true;
  double min = INF;
  double distance = 0.0;
  Coordinates centerCoords;
  set<string> unreachable;  
    

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
  

    if (searchBuilding(Buildings, input1Building, person1Building) == false) {
      cout << endl << "Person 1's building not found" << endl;
      found = false;
    } else if (searchBuilding(Buildings, input2Building, person2Building) == false) {
      cout << endl << "Person 2's building not found" << endl;
      found = false;
    }

    if (found == false) {
      cout << endl << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    }

    while (true) { // while true = path is not found
      if (unreachable.size() == Buildings.size()) {
        break;
      }

      centerCoords = centerBetween2Points(input1Building.Coords.Lat, input1Building.Coords.Lon, input2Building.Coords.Lat, input2Building.Coords.Lon);

      for (auto b : Buildings) {
        if (unreachable.count(b.Fullname) == true) {
          continue;
        }

        distance = distBetween2Points(centerCoords.Lat, centerCoords.Lon, b.Coords.Lat, b.Coords.Lon);
        if (distance < min) {
          min = distance;
          centerBuilding = b;
        }
      }
      
      long long startNode;
      long long endNode;
      long long centerNode;

      startNode = nearestNode(Footways, Nodes, input1Building);
      endNode = nearestNode(Footways, Nodes, input2Building);
      centerNode = nearestNode(Footways, Nodes, centerBuilding);

      map <long long, double> point1;
      map <long long, double> point2;
      map <long long, long long> pred1;
      map <long long, long long> pred2;

      DijkstraShortestPath(startNode, G, Nodes, pred1, point1);
      DijkstraShortestPath(endNode, G, Nodes, pred2, point2);

      if (unreachable.size() == 0) {
        cout << endl << "Person 1's point: " << endl;
        cout << " " << input1Building.Fullname << endl;
        cout << " (" << input1Building.Coords.Lat << ", " << input1Building.Coords.Lon << ")" << endl;

        cout << "Person 2's point: " << endl;
        cout << " " << input2Building.Fullname << endl;
        cout << " (" << input2Building.Coords.Lat << ", " << input2Building.Coords.Lon << ")" << endl;

        cout << "Destination Building: " << endl;
        cout << " " <<  centerBuilding.Fullname << endl;
        cout << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")" << endl;

        cout << "Nearest P1 node: " << endl;
        cout << " " <<  startNode << endl;
        cout << " (" << Nodes.at(startNode).Lat << ", " << Nodes.at(startNode).Lon << ")" << endl;
        
        cout << "Nearest P2 node: " << endl;
        cout << " " <<  endNode << endl;
        cout << " (" << Nodes.at(endNode).Lat << ", " << Nodes.at(endNode).Lon << ")" << endl;
                
        cout << "Nearest destination node: " << endl;
        cout << " " <<  centerNode << endl;
        cout << " (" << Nodes.at(centerNode).Lat << ", " << Nodes.at(centerNode).Lon << ")" << endl;
        
      } else {
        cout << "New destination building: " << endl;
        cout << " " << centerBuilding.Fullname << endl;
        cout << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")" << endl;

        cout << "Nearest destination node: " << endl;
        cout << " " << centerNode << endl;
        cout << " (" << Nodes.at(centerNode).Lat << ", " << Nodes.at(centerNode).Lon << ")" << endl;
      }

      if (pred2.at(startNode) >= INF) {
        cout << "Sorry, destination unreachable." << endl;
        break;
      }

      if(pred1.at(centerNode) >= INF || pred2.at(centerNode) >= INF) {
        cout << endl << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
        unreachable.insert(centerBuilding.Fullname);
        continue;
      }

      vector <long long> path1;
      vector <long long> path2;

      cout << endl << "Person 1's distance to dest: " << point1.at(centerNode) << " miles" << endl;
      path1 = getPath(pred1, centerNode);
      printPath(path1);

      cout << endl << "Person 2's distance to dest: " << point2.at(centerNode) << " miles" << endl;
      path2 = getPath(pred2, centerNode);
      printPath(path2);

      break;
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}


int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  //
  // DONE: build the graph, output stats:
  //
  for (auto a : Nodes) {
    G.addVertex(a.first);
  }

  for (auto const &b : Footways) {
    for (int i = 0; i < ((int)b.Nodes.size() - 1); ++i) {
      G.addEdge(b.Nodes[i], b.Nodes[i + 1], 
                distBetween2Points(Nodes.at(b.Nodes[i]).Lat, Nodes.at(b.Nodes[i]).Lon, Nodes.at(b.Nodes[i + 1]).Lat,                                          Nodes.at(b.Nodes[i + 1]).Lon));
      G.addEdge(b.Nodes[i + 1], b.Nodes[i], 
                distBetween2Points(Nodes.at(b.Nodes[i]).Lat, Nodes.at(b.Nodes[i]).Lon, Nodes.at(b.Nodes[i + 1]).Lat,                                          Nodes.at(b.Nodes[i + 1]).Lon));
    }
  }


  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
