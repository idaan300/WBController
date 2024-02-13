#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <list>

#include "Pathing.h"

//------------------------
//   Helper Functions
//------------------------

// Manhattan distance heuristic
int heuristic(const Node& current, const Node& target) {
    return abs(current.x - target.x) + abs(current.y - target.y);
}

// Check if the given coordinates are valid within the grid
bool isValid(int x, int y, int size) {
    return (x >= 0 && x < size && y >= 0 && y < size);
}

// Check if the node is already in the open or closed set
bool isNodeInSet(const std::vector<Node*>& set, const Node* node) {
    for (const auto& n : set) {
        if (n->x == node->x && n->y == node->y) {
            return true;
        }
    }
    return false;
}

//---------------------------------------
//   A* Path Planning Algo Main Funcs
//---------------------------------------

const std::vector<Node> GeneratePath(Node* node) {
    std::vector<Node*> pathptr;

    //Count the amount of entries in the vector
    int VecSize = 0;

    while (node != nullptr) {
        pathptr.push_back(node);
        node = node->parent;
        VecSize++;
    }

    //Construct a new Blank Vecor of Node Type (And Not Node*)
    std::vector<Node> path(VecSize - 1);

    //Algo Returns List in reverse, Unreverse it
    std::reverse(pathptr.begin(), pathptr.end());

    //Get The Actual Value from the NodePtr
    //becase the second we leave this function all of the Node* get invalidated
    for (int i = 0; i < VecSize - 1; i++) 
        memcpy(&path[i], (pathptr[i + 1]), sizeof(Node));
    
    return path;
}

// A* pathfinding algorithm
const std::vector<Node> FindPath(const std::vector<std::vector<int>>& grid, const Node& start, const Node& target) {
    const int size = grid.size();

    // Create a 2D array of nodes to represent the grid
    std::vector<std::vector<Node>> nodes(size, std::vector<Node>(size, Node(0, 0)));

    // Initialize the nodes with their coordinates
    for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size; ++y) {
            nodes[x][y].x = x;
            nodes[x][y].y = y;
        }
    }

    // Set the start node properties
    Node* currentNode = &nodes[start.x][start.y];
    currentNode->g = 0;
    currentNode->h = heuristic(*currentNode, target);

    // Create the open and closed sets
    std::vector<Node*> openSet;
    std::vector<Node*> closedSet;

    // Add the start node to the open set
    openSet.push_back(currentNode);

    // Start the A* algorithm
    while (!openSet.empty()) {
        // Find the node with the lowest f() score in the open set
        currentNode = openSet[0];
        int currentIndex = 0;
        for (int i = 1; i < openSet.size(); ++i) {
            if (openSet[i]->f() < currentNode->f()) {
                currentNode = openSet[i];
                currentIndex = i;
            }
        }

        // Remove the current node from the open set
        openSet.erase(openSet.begin() + currentIndex);

        // Add the current node to the closed set
        closedSet.push_back(currentNode);

        // Check if we have reached the target node
        if (currentNode->x == target.x && currentNode->y == target.y) {
            return GeneratePath(currentNode);
        }

        // Generate the neighboring nodes
        std::vector<Node*> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) {
                    continue;  // Ignore diagonal and center nodes
                }
                int nx = currentNode->x + dx;
                int ny = currentNode->y + dy;
                if (isValid(nx, ny, size) && grid[nx][ny] != 1) {
                    neighbors.push_back(&nodes[nx][ny]);
                }
            }
        }

        // Process the neighboring nodes
        for (auto neighbor : neighbors) {
            // Check if the neighbor is already in the closed set
            if (isNodeInSet(closedSet, neighbor)) {
                continue;
            }

            // Calculate the tentative g score
            int tentativeG = currentNode->g + 1;

            // Check if the neighbor is already in the open set
            bool isNewPath = false;
            if (!isNodeInSet(openSet, neighbor)) {
                // Add the neighbor to the open set
                openSet.push_back(neighbor);
                neighbor->h = heuristic(*neighbor, target);
                isNewPath = true;
            }
            else if (tentativeG < neighbor->g) {
                isNewPath = true;
            }

            // Update the neighbor's properties if we found a better path
            if (isNewPath) {
                neighbor->parent = currentNode;
                neighbor->g = tentativeG;
            }
        }
    }

    // No path found
    return {};
}


int Test() {

    std::cout << "Init Test \n";

    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };


    std::vector<Node> targets = {
    Node(1, 1),  // Target 1
    Node(5, 5),  // Target 1
    Node(2, 2),  // Target 2
    Node(8, 10)  // Target 3
    };

    Node start(0, 0);

    std::vector<Node> path;

    // Traverse each target linearly
    for (const auto& target : targets) {
        std::vector<Node> partialPath = FindPath(grid, start, target);



        if (!partialPath.empty()) {
            path.insert(path.end(), partialPath.begin(), partialPath.end());
            start = target;  // Update the start position for the next target
        }
    }

    if (!path.empty()) {
        std::cout << "Path found!\n";
        for (const auto& node : path) {
            std::cout << "(" << node.x << ", " << node.y << ") ";
        }
        std::cout << "\n";
    }
    else {
        std::cout << "No path found.\n";
    }

    return 0;
}
