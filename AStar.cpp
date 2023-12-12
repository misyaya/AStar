#include "AStar.h"
#include <iostream>
#include <queue>


using std::cout;
using std::cin;
using std::endl;

Node::Node(int x, int y) :
	x(x), y(y), g(0), h(0), parent(nullptr)
{
}

int Node::getF() const
{
	return g + h;
}

int AStar::calculateH(Node* a, Node* b)
{
	return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

vector<Node*> AStar::aStar(Node* start, Node* goal, vector<vector<Node*>>& graph)
{
	 std::vector<Node*> path;
    std::priority_queue<std::pair<int, Node*>, std::vector<std::pair<int, Node*>>, std::greater<std::pair<int, Node*>>> openSet;

    openSet.push(std::make_pair(0, start));

    while (!openSet.empty()) {
        Node* current = openSet.top().second;
        openSet.pop();

        if (current == goal) {
            Node* node = current;
            while (node != nullptr) {
                path.push_back(node);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (Node* neighbor : graph[current->x][current->y]->neighbors) {
            int tentativeG = current->g + 1;

            if (tentativeG < neighbor->g) {
                neighbor->parent = current;
                neighbor->g = tentativeG;
                neighbor->h = calculateH(neighbor, goal);
                openSet.push(std::make_pair(neighbor->getF(), neighbor));
            }
        }
    }

    return path;
}
