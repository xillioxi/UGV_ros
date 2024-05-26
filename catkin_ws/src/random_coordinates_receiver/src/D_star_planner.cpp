#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <queue>
#include <fstream>
#include <iomanip>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

// Constants
const double INF = std::numeric_limits<double>::infinity();
bool point_received = false;
geometry_msgs::Point received_point;

struct Node {
    std::string state = "OPEN"; // Possible states: OPEN, CLOSED, ERROR, BLOCKED
    double cost = INF;
    int x, y;
    Node* parent = nullptr;
    std::vector<Node*> children;
    double heuristic = INF;

    Node(int x, int y) : x(x), y(y) {}

    void print() {
        std::cout << "<Node x:" << x << " y:" << y << " state:" << state << ">\n";
    }
};

struct Map {
    std::vector<std::vector<Node>> map;
    Node* goal;
    int size_x, size_y;

    Map(int size_x, int size_y, int goal_x, int goal_y) : size_x(size_x), size_y(size_y) {
        map.resize(size_y, std::vector<Node>(size_x, Node(0, 0)));
        for (int y = 0; y < size_y; ++y) {
            for (int x = 0; x < size_x; ++x) {
                map[y][x] = Node(x, y);
            }
        }
        goal = &map[goal_y][goal_x];
    }

    void print() {
        for (auto& row : map) {
            for (Node& node : row) {
                std::cout << round(node.cost * 1000) / 1000 << "\t";
            }
            std::cout << "\n";
        }
    }

    void printState() {
        for (auto& row : map) {
            for (Node& node : row) {
                if (node.state == "OPEN")
                    std::cout << "O\t";
                else if (node.state == "CLOSED")
                    std::cout << "C\t";
                else if (node.state == "BLOCKED")
                    std::cout << "B\t";
            }
            std::cout << "\n";
        }
    }

    double getCost(int x, int y) {
        return map[y][x].cost;
    }

    void block(int x, int y) {
        map[y][x].state = "BLOCKED";
        map[y][x].cost = -1;
        reset(map[y][x]);
    }

    void reset(Node& node) {
        if (node.state != "BLOCKED") {
            node.state = "OPEN";
            node.cost = INF;
        }
        for (Node* child : node.children) {
            reset(*child);
        }
    }
};

struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->cost > b->cost;
    }
};

class PriorityQueue {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> queue;

public:
    void push(Node* node) {
        queue.push(node);
    }

    Node* pop() {
        Node* node = queue.top();
        queue.pop();
        return node;
    }

    bool isEmpty() const {
        return queue.empty();
    }
};

void expand(Node* node, Map& map, PriorityQueue& nodesQueue) {
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            if (i == 0 && j == 0) continue; // Skip the current node itself
            int nx = node->x + i;
            int ny = node->y + j;
            if (nx >= 0 && ny >= 0 && nx < map.size_x && ny < map.size_y) {
                Node* nextNode = &map.map[ny][nx];
                double newCost = heuristic(node, i, j, map);
                if (newCost < nextNode->cost && nextNode->state != "CLOSED" && nextNode->state != "BLOCKED") {
                    nextNode->state = "CLOSED";
                    nextNode->cost = newCost;
                    nextNode->parent = node;
                    node->children.push_back(nextNode);
                    nodesQueue.push(nextNode);
                }
            }
        }
    }
}

double heuristic(Node* node, int i, int j, Map& map) {
    return sqrt(i*i + j*j) + node->cost; // Ensure this matches Python's calculation
}

void initiate(Map& map) {
    map.goal->cost = 0;
    PriorityQueue nodesQueue;
    nodesQueue.push(map.goal);
    while (!nodesQueue.isEmpty()) {
        expand(nodesQueue.pop(), map, nodesQueue);
    }
}

void saveMapToFile(const Map& map, const std::string& filename = "map_output.txt") {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing.\n";
        return;
    }

    for (const auto& row : map.map) {
        for (const Node& node : row) {
            file << std::fixed << std::setprecision(3) << node.cost << "\t";
        }
        file << "\n";
    }
}

double manhattan(const Node& goal_node, const Node& node) {
    return sqrt(pow(goal_node.x - node.x, 2) + pow(goal_node.y - node.y, 2));
}

bool pathBlocked(const Node* node) {
    if (node == nullptr) return false;
    if (node->cost == 0) return false;
    if (node->state == "BLOCKED") {
        std::cout << "PATH BLOCKED!!\n";
        if (node->parent) std::cout << "Parent x: " << node->parent->x << " y: " << node->parent->y << "\n";
        return true;
    }
    return pathBlocked(node->parent);
}

void pointCallback(const geometry_msgs::Point::ConstPtr& msg) {
    received_point = *msg;
    point_received = true;
}

geometry_msgs::Point waitForPoint() {
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("goal_location", 1, pointCallback);
    ros::Rate rate(10); // Check the callback at 10 Hz

    int timeout = 100; // Timeout after 10 seconds (100/10)
    while (ros::ok() && !point_received && timeout > 0) {
        ros::spinOnce();
        rate.sleep();
        timeout--;
    }

    if (!point_received) {
        ROS_WARN("No point received within the timeout period.");
    }

    return received_point;
}

int main() {geometry_msgs::Point point = waitForPoint();

    int size_x = 10;
    int size_y = 10;
    int goal_x = 9;
    int goal_y = 1;
    int start_x = 3;
    int start_y = 0;

    Map map(size_x, size_y, goal_x, goal_y);
    map.block(5, 1);
    map.block(5, 2);
    map.block(5, 3);
    map.block(6, 1);
    map.printState();

    initiate(map);
    Node* startNode = &map.map[start_y][start_x];
    std::cout << "Path blocked from start? " << (pathBlocked(startNode) ? "Yes" : "No") << "\n";
    map.print();

    saveMapToFile(map);

    return 0;
}
