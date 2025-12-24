#pragma once
#include <string>
#include <unordered_map>
#include <vector>

struct Node {
        std::string name;
        std::string city;
        std::string code;

        Node() = default;
        Node(std::string name, std::string city, std::string code)
            : name(name),
              city(city),
              code(code) {}

        std::string getName() const { return name; }
        std::string getCity() const { return city; }
        std::string getCode() const { return code; }
};

struct Edge {
        std::string to;
        double distance;
        double time;
        double cost;

        Edge() = default;
        Edge(std::string to, double distance, double time, double cost)
            : to(to),
              distance(distance),
              time(time),
              cost(cost) {}
};

using adj = std::unordered_map<std::string, std::vector<Edge>>;

// Function declarations

std::vector<Node> findShortestPath(
    Node start, Node end, std::unordered_map<std::string, int> userPriority,
    adj& graph, std::unordered_map<std::string, Node>& nodes);

std::vector<std::vector<std::string>> findSCC(adj& graph);

bool pathExists(std::string start, std::string target, adj& graph);

std::string findKthShortestPath(adj& graph,
                                std::unordered_map<std::string, Node>& nodes,
                                const std::string& src, const std::string& dst,
                                int k);

std::vector<Edge> mostCriticalEdges(adj& graph);

std::vector<std::vector<std::string>> suggestHubConnection(adj& graph);

std::vector<std::string> reachableNodes(Node start, adj& graph);
