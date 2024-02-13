#pragma once

//a* Pathing Algo Node Type
struct Node {
    int x, y;
    int g, h;
    Node* parent;

    Node(int x = -1, int y = -1) : x(x), y(y), g(0), h(0), parent(nullptr) {}

    int f() const {
        return g + h;
    }
};

const std::vector<Node> FindPath(const std::vector<std::vector<int>>& grid, const Node& start, const Node& target);
int Test();