#include <algorithm>
#include <stdexcept>
#include "Graph.h"

using std::find;
using std::sort;
using std::invalid_argument;
using std::priority_queue;

// **COMPLETE THIS SECTION**
// Graph Algorithms------------------------------------------------

	//PRE: startKey is in the graph
	//PARAM: startKey - the key of the start vertex for BFS
	//POST: performs a breadth-first search from startKey, returns a queue
	//      containing the keys of the vertices in the order they were visited	
queue<int> Graph::bfs(int startKey) const
{
	// To Do: Complete this method
    // 1. Make an empty queue<int> result
    // 2. Make a visited unordered_map<int, bool>
    // 3. Make a queue<int> toVisit and push startKey
    // 4. Mark startKey visited = true
    // 5. While toVisit is not empty:
    //      a. Let v = toVisit.front(), pop it
    //      b. Push v into result queue
    //      c. For each Edge e in edges.at(v):
    //           - If e.toKey not visited:
    //                * Mark visited = true
    //                * Push e.toKey into toVisit
    // 6. Return result queue
}

//PRE: startKey is in the graph
//PARAM: startKey - the key of the start vertex for DFS
//POST: performs a depth-first search from startKey, returns a queue
//      containing the keys of the vertices in the order they were visited	
queue<int> Graph::dfs(int startKey) const
{
	// To Do: Complete this method
    // 1. Make an empty queue<int> result
    // 2. Make a visited unordered_map<int, bool>
    // 3. Make a stack<int> toVisit and push startKey
    // 4. While stack is not empty:
    //      a. Let v = stack.top(), pop it
    //      b. If v not visited:
    //            * Mark visited = true
    //            * Push v into result queue
    //      c. For each Edge e in edges.at(v) (optional: traverse list in reverse):
    //            - If e.toKey not visited:
    //                * Push e.toKey into the stack
    // 5. Return result queue
}

//PRE: startKey and endKey are in the graph
//PARAM: startKey - the key of the start vertex for Dijkstra's algorithm
//       endKey - the key of the end vertex for Dijkstra's algorithm
//POST: calls  dijkstra to generate shortest paths from startKey to
//      endKey, returns a stack containing the keys of the vertices in the
//      order they appear in the shortest path from startKey to endKey	
stack<int> Graph::getShortestPath(int startKey, int endKey, int& cost) const
{
	// To Do: Complete this method
    // 1. Call dijkstra(startKey) -> store in map paths
    // 2. Create empty stack<int> result
    // 3. Let cur = endKey
    // 4. totalCost = paths.at(endKey).cost
    // 5. While cur != -1:
    //        Push cur into result stack
    //        cur = paths.at(cur).parentKey
    // 6. Set cost = totalCost
    // 7. Return result stack
}

// Helper method for getShortestPath
	//PRE: startKey is in the graph
	//PARAM: startKey - the key of the start vertex for Dijkstra's algorithm
	//POST: generates the shortest path from the start vertex to all other 
	//      connected vertices in the graph using Dijkstra's algorithm, returns
	//      an unordered_map where the key is the vertex key and the value is
	//      a pathNode structure containing the cost to that node and its parent
unordered_map<int, SPResultRecord> Graph::dijkstra(int startKey) const
{
	// To Do: Complete this method
    // 1. Make unordered_map<int, SPResultRecord> result
    // 2. Make a min-heap priority queue:
    //      priority_queue<PathRecord, vector<PathRecord>, PathRecordComparator> toVisit
    // 3. Make unordered_map<int, int> lowestCost
    // 4. Insert initial record:
    //      PathRecord(startKey, 0, -1) into toVisit
    //      lowestCost[startKey] = 0
    // 5. While toVisit not empty:
    //      a. Pop top record -> call it cur
    //      b. If cur.vKey already in result, skip (continue)
    //      c. Add cur.vKey to result:
    //            result[cur.vKey] = SPResultRecord(cur.cost, cur.parentKey)
    //      d. For each Edge e in edges.at(cur.vKey):
    //            newCost = cur.cost + e.weight
    //            If newCost < lowestCost[e.toKey]:
    //                * lowestCost[e.toKey] = newCost
    //                * Push PathRecord(e.toKey, newCost, cur.vKey) into toVisit
    // 6. Return result map
}

//PRE:
//POST: generates the minimum spanning tree of the graph using Krsukal's
//      algorithm, returns a queue containing the edges in the MST
queue<Edge> Graph::mst() const
{
	// To Do: Complete this method
    // 1. Copy all edges into a vector<Edge> allEdges (only one direction per edge, avoid duplicates)
    // 2. Sort allEdges using sort()  (smallest weight first)
    // 3. Create 2 maps for disjoint sets:
    //      unordered_map<int, unordered_set<int>> sets
    //      unordered_map<int, int> setID
    // 4. For each vertex key v in vertices map:
    //        Make a new set containing just v
    //        sets[v] = {v}
    //        setID[v] = v
    // 5. Create empty queue<Edge> result
    // 6. For each Edge e in sorted allEdges:
    //      a. If setID[e.fromKey] == setID[e.toKey], skip (cycle)
    //      b. Else union sets:
    //            old = setID[e.toKey]
    //            for each x in sets[old]:
    //                 sets[setID[e.fromKey]].insert(x)
    //                 setID[x] = setID[e.fromKey]
    //            erase sets[old]
    //      c. Push e into result queue
    //      d. Stop when result size = numVertices - 1
    // 7. Return result queue
}

// **END SECTION TO BE COMPLETED**---------------------------------
// 
// Constructors
Graph::Graph()
{
	numVertices = 0;
	numEdges = 0;
}

Graph::Graph(string fname)
{
	numVertices = 0;
	numEdges = 0;
	// NOT IMPLEMENTED
}

// Vertex and Edge insertion/removal methods-----------------------
void Graph::insertVertex(int key, string name)
{
	vertices.insert({ key, name });
	numVertices++;
}

void Graph::insertEdge(int v1, int v2, int cost)
{
	if(v1 == v2) {
		throw invalid_argument("error: self edge");
	}
	if (!hasVertex(v1) || !hasVertex(v2)) {
		throw invalid_argument("error: from / to vertex does not exist");
	}
	if(hasEdge(v1, v2)) {
		throw invalid_argument("error: edge already in graph");
	}

	edges[v1].push_back(Edge(v1, v2, cost));
	edges[v2].push_back(Edge(v2, v1, cost));
	numEdges++;
}

void Graph::loadBFSTestGraph()
{
	for(int i = 0; i < 12; i++) {
		insertVertex(i, "v" + std::to_string(i));
	}
	insertEdge(0, 1, 1);
	insertEdge(1, 2, 1);
	insertEdge(2, 3, 1);
	insertEdge(0, 5, 1);
	insertEdge(4, 5, 1);
	insertEdge(4, 7, 1);
	insertEdge(5, 6, 1);
	insertEdge(5, 7, 1);
	insertEdge(6, 9, 1);
	insertEdge(7, 8, 1);
	insertEdge(7, 10, 1);
	insertEdge(8, 10, 1);
	insertEdge(8, 9, 1);
	insertEdge(9, 11, 1);
}

void Graph::loadMSTTestGraph()
{
	for (int i = 0; i < 9; i++) {
		insertVertex(i, "v" + std::to_string(i));
	}
	insertEdge(0, 1, 8);
	insertEdge(0, 2, 4);
	insertEdge(1, 2, 10);
	insertEdge(1, 3, 1);
	insertEdge(1, 4, 9);
	insertEdge(2, 4, 19);
	insertEdge(2, 5, 12);
	insertEdge(3, 4, 6);
	insertEdge(3, 6, 3);
	insertEdge(4, 5, 2);
	insertEdge(5, 6, 5);
	insertEdge(5, 7, 7);
	insertEdge(6, 7, 14);
	insertEdge(6, 8, 11);
	insertEdge(7, 8, 13);
}

void Graph::removeEdge(int v1, int v2)
{
	// NOT IMPLEMENTED
}

void Graph::removeVertex(int v)
{
	// NOT IMPLEMENTED
}

// Vertex and Edge Query Methods-----------------------------------
int Graph::getNumVertices() const
{
	return numVertices;
}

int Graph::getNumEdges() const
{
	return numEdges;
}

bool Graph::empty() const
{
	return numVertices == 0;
}

bool Graph::hasVertex(int key) const
{
	return(vertices.find(key) != vertices.end());
}

bool Graph::hasEdge(int v1, int v2) const
{
	Edge target(v1, v2, 0);
	if (edges.find(v1) != edges.end() &&
		find(edges.at(v1).begin(), edges.at(v1).end(), target) != edges.at(v1).end()) {
		
		return true;
	}
	return false;
}

list<Edge> Graph::getNeigbours(int v) const
{
	return edges.at(v);
}

void Graph::print() const
{
	for (auto & ls : edges) {
		cout << ls.first << " (" <<  vertices.at(ls.first) << "): ";
		for (const Edge & e : ls.second) {
			cout << "{" << e.toKey << ' ' << e.weight << "} ";
		}
		cout << endl;
	}
}

// Graph Algorithm Print Methods For Testing-----------------------

//PRE: startKey is in the graph
//PARAM: startKey - the key of the start vertex for BFS
//POST: prints bfs from start
void Graph::printBFS(int startKey) const
{
	queue<int> q = bfs(startKey);

	cout << "BFS: ";
	while(!q.empty()) {
		cout << q.front() << " ";
		q.pop();
	}
}

//PRE: startKey is in the graph
//PARAM: startKey - the key of the start vertex for DFS
//POST: prints dfs from start
void Graph::printDFS(int startKey) const
{
	queue<int> q = dfs(startKey);

	cout << "DFS: ";
	while (!q.empty()) {
		cout << q.front() << " ";
		q.pop();
	}
}

//PRE: startKey and endKey are in the graph
//PARAM: startKey - the key of the start vertex for Dijkstra's algorithm
//       endKey - the key of the end vertex for Dijkstra's algorithm
//POST:  prints shortest path from start to end
void Graph::printShortestPath(int startKey, int endKey) const
{
	int totalCost;
	stack<int> sPaths = getShortestPath(startKey, endKey, totalCost);

	while(!sPaths.empty()) {
		cout << sPaths.top() << " ";
		sPaths.pop();
	}
	cout << endl << "Total Cost = " << totalCost << endl;
}

//PRE:
//POST: prints minimum spanning treein edge weight ascending order
void Graph::printMST() const
{
	queue<Edge> mstEdges = mst();

	while (!mstEdges.empty()) {
        cout << "from " << mstEdges.front().fromKey;
        cout << " to " << mstEdges.front().toKey;
        cout << ", weight = " << mstEdges.front().weight << endl;
        mstEdges.pop();
    }
}