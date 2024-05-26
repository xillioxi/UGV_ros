#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <queue>
#include <limits>
#include <functional>
#include <memory>
#include <algorithm>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <thread>
#include <chrono>

const double INF = std::numeric_limits<double>::infinity();
bool point_received = false;
geometry_msgs::Point received_point;

// Forward declarations
class Node;
class Map;
class PriorityQueue;
class PriorityQueue_2;

// Function declarations
double heuristic(Node* node, int i, int j, const Map& map);
void expand(Node* node, Map& map, PriorityQueue& nodesQueue);
bool expand_reverse(Map& map, Node* node, PriorityQueue_2& queue);
double manhattan(const Node& goal_node, const Node& node);
bool path_blocked(const Node* node);
void replanner(Map& map, Node* node);
void initiate(Map& map);
double expanddistance(Node* node);
static int percentageGenerated = 0;
Map* map = nullptr;

class Node {
public:
    std::string state;
    double cost;
    int x, y;
    Node* parent;
    std::vector<Node*> children;
    double heuristic;

    Node(int x, int y) : state("OPEN"), cost(INF), x(x), y(y), parent(nullptr), heuristic(INF) {}

    void print() {
        std::cout << "<Node x:" << x << " y:" << y << " state:" << state << ">\n";
    }
};

class Map {
public:
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


    void block(int x, int y) {
        if(map[y][x].state != "BLOCKED"){
            map[y][x].state = "BLOCKED";
            map[y][x].cost = -1;
            reset(&map[y][x]);
        }
    }

    static void reset(Node* node) {
        if (node->state != "BLOCKED") {
            node->state = "OPEN";
            node->cost = INF;
        }
        for (auto& child : node->children) {
            reset(child);
        }
    }
};

class PriorityQueue {
public:
    std::vector<Node*> nodes;

    void push(Node* node) {
        auto it = std::find_if(nodes.begin(), nodes.end(), [node](const Node* n) { return node->cost < n->cost; });
        nodes.insert(it, node);
    }

    Node* pop() {
        if (!nodes.empty()) {
            Node* node = nodes.front();
            nodes.erase(nodes.begin());
            return node;
        }
        return nullptr;
    }

    bool isEmpty() const {
        return nodes.empty();
    }
};



class PriorityQueue_2 {
public:
    std::vector<Node*> nodes;

    void push(Node* node) {
        auto it = std::find_if(nodes.begin(), nodes.end(), [node](const Node* n) { return node->heuristic < n->heuristic; });
        nodes.insert(it, node);
    }

    Node* pop() {
        if (!nodes.empty()) {
            Node* node = nodes.front();
            nodes.erase(nodes.begin());
            return node;
        }
        return nullptr;
    }

    bool isEmpty() const {
        return nodes.empty();
    }
};

double heuristic(Node* node, int i, int j, const Map& map) {
    return std::sqrt(i * i + j * j) + node->cost;
}

void expand(Node* node, Map& map, PriorityQueue& nodesQueue) {
    if(percentageGenerated %100000 == 0){
        std::cout << "Initializing:" << percentageGenerated/10000 << "%" << std::endl << std::flush;;
    }percentageGenerated+=1;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            if (i == 0 && j == 0) continue;
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

void initiate(Map& map) {
    map.goal->cost = 0;
    PriorityQueue nodesQueue;
    nodesQueue.push(map.goal);
    while (!nodesQueue.isEmpty()) {
        expand(nodesQueue.pop(), map, nodesQueue);
    }
}

bool expand_reverse(Map& map, Node* node, PriorityQueue_2& queue) {
    node->print(); 
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            if (i == 0 && j == 0) continue;
            int nx = node->x + i;
            int ny = node->y + j;
            if (nx >= 0 && ny >= 0 && nx < map.size_x && ny < map.size_y) {
                Node* nextNode = &map.map[ny][nx];
                if (nextNode->state != "CLOSED" && nextNode->state != "BLOCKED") {
                    nextNode->heuristic = manhattan(*map.goal, *nextNode);
                    nextNode->parent = node;
                    node->children.push_back(nextNode);
                    queue.push(nextNode);
                }
                if (nextNode->state == "CLOSED") {
                    node->parent = nextNode;
                    nextNode->children.push_back(node);
                    return true;
                }
            }
        }
    }
    return false;
}

double expanddistance(Node* node) {
    std::cout << "here\n";
    if (node->state == "CLOSED") {
        return node->cost;
    } else {
        double distance = manhattan(*node, *node->parent);
        node->cost = expanddistance(node->parent) + distance;
        std::cout << node->cost << std::endl;
        return node->cost;
    }
}

double manhattan(const Node& goal_node, const Node& node) {
    return std::sqrt((goal_node.x - node.x) * (goal_node.x - node.x) +
                     (goal_node.y - node.y) * (goal_node.y - node.y));
}

bool path_blocked(const Node* node) {
    if (node == nullptr) return false;
    if (node->cost == 0) return false;
    if (node->state == "BLOCKED") {
        std::cout << "PATH BLOCKED!!\n";
        if (node->parent) {
            std::cout << "Parent x: " << node->parent->x << " y:" << node->parent->y << "\n";
        }
        return true;
    }
    return path_blocked(node->parent);
}

void replanner(Map& map, Node* node) {
    if (path_blocked(node)) {
        bool found = false;
        PriorityQueue_2 queue;
        queue.push(node);
        while (!queue.isEmpty() && !found) {
            Node* current = queue.pop();
            found = expand_reverse(map, current, queue);
        }
    }
    expanddistance(node);
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
        std::cout << "No goal received within the timeout period.";
    }
    std::cout << "Goal Received" << std::flush;;
    ROS_INFO("You skipped me!");

    return received_point;
}

void pointCallback2(const geometry_msgs::Point::ConstPtr& msg) {
    received_point = *msg;
    map->block(received_point.x,received_point.y);
}

int main(int argc, char** argv) {
    ROS_INFO("HELLO THERE");
    ros::init(argc, argv, "point_listener");
    geometry_msgs::Point point = waitForPoint();
    int size_x = 1000;
    int size_y = 1000;
    std::cout << "Goal Received" << std::flush;;
    int goal_x = point.x;
    int goal_y = point.y;
    int start_x = 500;
    int start_y = 500;

    map = new Map(size_x, size_y, goal_x, goal_y);
    std::cout << "Map Generated" << std::flush;


    initiate(*map);
    std::cout << "Map Initialized" << std::flush;

    Node* startNode = &map->map[start_y][start_x];
    /*
    map.block(5, 1);
    map.block(5, 2);
    map.block(5, 3);
    map.block(6, 1);
    */
    ros::Rate rate(5.0);

    ros::init(argc, argv, "node_blocker");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("blocked_nodes", 1000, pointCallback2);
    
    while (ros::ok()) {
        if (path_blocked(startNode)) {
            std::cout << "Path blocked from start? " << (path_blocked(startNode) ? "Yes" : "No") << "\n" << std::endl << std::flush;
            replanner(*map, startNode);
        }

        // Sleep to maintain the loop rate
        rate.sleep();
    }


    return 0;
}