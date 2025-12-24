#include <iostream>
#include "airline_graph.cpp"


using namespace std;
int main() {
    unordered_map<string, Node> airlines;

    // register cities
    while (true) {
        string name, city, code;
        cin >> name >> city >> code;

        if (airlines.find(code) != airlines.end())
            throw runtime_error("code " + code + " already exists");
        airlines[code] = Node(name, city, code);
    }

    adj adjacency;
    set<pair<string, string>> existingEdges;

    // register lines
    while (true) {
        // start and to are airport codes
        string start, to;
        float distance, time, cost;
        cin >> start >> to >> distance >> time >> cost;
        if (airlines.find(start) == airlines.end())
            throw runtime_error("code " + start + " does not exist");

        if (airlines.find(to) == airlines.end())
            throw runtime_error("code " + to + " does not exist");

        if (existingEdges.count({start, to}))
            throw runtime_error("edge between " + start + " " + to + " " +
                                "already exists");
        Edge edge(to, distance, time, cost);
        adjacency[start].push_back(edge);
        existingEdges.insert({start, to});
    }
}
