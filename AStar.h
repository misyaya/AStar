#pragma once
#include <vector>
using std::vector;

struct Node {
	int x, y;
	int g, h;
	Node* parent;
	vector<Node*> neighbors;
	Node(int x, int y);
	int getF() const;
};

class AStar
{
public:
	int calculateH(Node* a, Node* b);
	vector<Node*> aStar(Node* start, Node* goal, vector<vector<Node*>>& graph);
};

