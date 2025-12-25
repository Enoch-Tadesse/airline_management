#include "airline_graph.h"

#include <algorithm>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

using namespace std;

/*
 * find the shortest path from a start node to end node
 * based on priority provided by the user
 * */
vector<Node> findShortestPath(
    Node start, Node end, unordered_map<string, int> userPriority, adj& graph,
    unordered_map<string, Node>&
        nodes)  // pass nodes map to get Node objects by code
{
    struct QueueEntry {
            string code;
            double distance;
            double time;
            double cost;
    };

    // mapping of index to name
    const vector<string> idxToName = {"distance", "time", "cost"};

    // comparator for priority queue using user defined priorities
    // which is a min heap
    auto cmp = [&](const QueueEntry& a, const QueueEntry& b) {
        for (int pr = 1; pr <= 3; ++pr) {
            for (int i = 0; i < 3; ++i) {
                if (userPriority[idxToName[i]] == pr) {
                    double valA, valB;
                    if (i == 0) {
                        valA = a.distance;
                        valB = b.distance;
                    } else if (i == 1) {
                        valA = a.time;
                        valB = b.time;
                    } else {
                        valA = a.cost;
                        valB = b.cost;
                    }
                    if (valA != valB) return valA > valB;
                }
            }
        }
        return false;
    };

    priority_queue<QueueEntry, vector<QueueEntry>, decltype(cmp)> pq(cmp);
    unordered_map<string, QueueEntry> best;
    unordered_map<string, string> previous;

    // initialize the priority_queue
    pq.push({start.getCode(), 0, 0, 0});
    best[start.getCode()] = {start.getCode(), 0, 0, 0};
    previous[start.getCode()] = "";

    while (!pq.empty()) {
        QueueEntry current = pq.top();
        pq.pop();

        if (current.code == end.getCode()) break;

        for (auto& edge : graph[current.code]) {
            QueueEntry next;
            next.code = edge.to;
            next.distance = current.distance + edge.distance;
            next.time = current.time + edge.time;
            next.cost = current.cost + edge.cost;

            // update if first time visiting or better according to lex order
            if (best.find(next.code) == best.end()) {
                best[next.code] = next;
                previous[next.code] = current.code;
                pq.push(next);
            } else {
                QueueEntry& old = best[next.code];
                // compare lexicographically according to priority
                bool better = false;
                for (int pr = 1; pr <= 3; ++pr) {
                    for (int i = 0; i < 3; ++i) {
                        string field = idxToName[i];
                        if (userPriority[field] == pr) {
                            double valNew = (i == 0   ? next.distance
                                             : i == 1 ? next.time
                                                      : next.cost);
                            double valOld = (i == 0   ? old.distance
                                             : i == 1 ? old.time
                                                      : old.cost);
                            if (valNew < valOld) {
                                better = true;
                                goto update;
                            } else if (valNew > valOld)
                                goto skip;
                        }
                    }
                }
            update:
                best[next.code] = next;
                previous[next.code] = current.code;
                pq.push(next);
            skip:;
            }
        }
    }

    // reconstruct path by tracking previous nodes parents back
    vector<Node> path;
    string curr = end.getCode();
    while (curr != "") {
        path.push_back(nodes[curr]);
        curr = previous[curr];
    }
    reverse(path.begin(), path.end());
    return path;
}

/*
 * find strongly connected components (nodes inside a single set that,
 * if you start from one you can reach every other node which applies for all
 * the nodes inside a single strongly connected components)
 * the funcition uses tarjan's algorithm
 * */
vector<vector<string>> findSCC(adj& graph) {
    unordered_map<string, int> ids;
    unordered_map<string, int> low;
    vector<string> stack;
    unordered_set<string> on_stack;

    int id = 0;
    int count = 0;

    auto tarjan = [&](auto& self, string current) -> void {
        on_stack.insert(current);
        stack.push_back(current);
        ++id;
        ids[current] = id;
        low[current] = id;
        for (Edge& edge : graph[current]) {
            string nei = edge.to;
            if (ids.find(nei) == ids.end()) {
                self(self, nei);
            }
            if (on_stack.find(nei) != on_stack.end()) {
                low[current] = min(low[current], low[nei]);
            }
        }

        if (low[current] == ids[current]) {
            while (true) {
                string top = stack.back();
                stack.pop_back();
                low[top] = min(low[top], low[current]);
                on_stack.erase(top);

                if (top == current) break;
            }
            count++;
        }
    };

    for (auto& [node, _] : graph) {
        if (ids.find(node) == ids.end()) tarjan(tarjan, node);
    }

    // collects all the scc component nodes into one
    unordered_map<int, vector<string>> collections;
    for (auto& [airline, i] : low) {
        collections[i].push_back(airline);
    }

    // collects scc as a vector<vector<string>>
    vector<vector<string>> scc;
    for (auto& pair : collections) scc.push_back(pair.second);

    return scc;
}

/*
 * this function checks whether we have a prth between two nodes or not
 * returns true if path exist otherwise it returns false
 * */
bool pathExists(string start, string target, adj& graph) {
    queue<string> q;
    q.push(start);
    unordered_map<string, bool> visited;

    while (!q.empty()) {
        string current = q.front();
        q.pop();
        visited[current] = true;

		// target found, return true
        if (current == target) return true;

        for (Edge edge : graph[current]) {
			// visit every edge that has not been visited before
            if (visited[edge.to]) continue;
            q.push(edge.to);
        }
    }
    return false;
}

/**
 * Computes the k-th shortest simple path between two nodes in a directed graph
 * using Yen’s k-shortest paths algorithm (with Dijkstra as the shortest-path
 * subroutine). Returns the path as a "A->B->C" string, or an empty string if
 * fewer than k paths exist.
 */
string findKthShortestPath(adj& graph, unordered_map<string, Node>& nodes,
                           const string& src, const string& dst, int k) {
    using Path = vector<string>;

    auto dijkstra = [&](const string& start, const string& end,
                        set<pair<string, string>>& bannedEdges,
                        set<string>& bannedNodes) -> Path {
        unordered_map<string, double> dist;
        unordered_map<string, string> parent;

        for (auto& [code, _] : nodes) dist[code] = 1e18;

        priority_queue<pair<double, string>, vector<pair<double, string>>,
                       greater<>>
            pq;

        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > dist[u]) continue;
            if (u == end) break;

            if (graph.count(u) == 0) continue;

            for (auto& e : graph[u]) {
                if (bannedNodes.count(e.to)) continue;
                if (bannedEdges.count({u, e.to})) continue;

                double nd = d + e.cost;
                if (nd < dist[e.to]) {
                    dist[e.to] = nd;
                    parent[e.to] = u;
                    pq.push({nd, e.to});
                }
            }
        }

        if (dist[end] == 1e18) return {};

        Path path;
        for (string cur = end; cur != start; cur = parent[cur])
            path.push_back(cur);
        path.push_back(start);

        reverse(path.begin(), path.end());
        return path;
    };

    vector<Path> A;  // shortest paths
    priority_queue<pair<double, Path>, vector<pair<double, Path>>,
                   greater<>>
        B;  // candidate paths

    set<pair<string, string>> noEdges;
    set<string> noNodes;

    Path first = dijkstra(src, dst, noEdges, noNodes);
    if (first.empty()) return "";

    A.push_back(first);

    auto pathCost = [&](const Path& p) {
        double cost = 0;
        for (int i = 0; i + 1 < (int)p.size(); ++i) {
            for (auto& e : graph[p[i]]) {
                if (e.to == p[i + 1]) {
                    cost += e.cost;
                    break;
                }
            }
        }
        return cost;
    };

    for (int i = 1; i < k; ++i) {
        const Path& prev = A[i - 1];

        for (int spur = 0; spur + 1 < (int)prev.size(); ++spur) {
            string spurNode = prev[spur];
            Path root(prev.begin(), prev.begin() + spur + 1);

            set<pair<string, string>> bannedEdges;
            set<string> bannedNodes;

            for (auto& p : A) {
                if ((int)p.size() > spur &&
                    equal(root.begin(), root.end(), p.begin())) {
                    bannedEdges.insert({p[spur], p[spur + 1]});
                }
            }

            for (int j = 0; j < spur; ++j) bannedNodes.insert(root[j]);

            Path spurPath = dijkstra(spurNode, dst, bannedEdges, bannedNodes);

            if (spurPath.empty()) continue;

            Path total = root;
            total.pop_back();
            total.insert(total.end(), spurPath.begin(), spurPath.end());

            B.push({pathCost(total), total});
        }

        if (B.empty()) break;

        A.push_back(B.top().second);
        B.pop();
    }

    if ((int)A.size() < k) return "";

    string result;
    for (int i = 0; i < (int)A[k - 1].size(); ++i) {
        if (i) result += "->";
        result += A[k - 1][i];
    }

    return result;
}

/**
 * Finds the edges in a directed graph whose removal increases the number
 * of strongly connected components the most. Returns a vector of such
 * critical edges. If no edge increases SCCs, returns an empty vector.
 */
vector<Edge> mostCriticalEdges(adj& graph) {
    int initialScc = findSCC(graph).size();
    int _max = initialScc;

    unordered_map<int, vector<Edge>> candidates;

    for (auto& [parent, edges] : graph) {
        for (int i = (int)edges.size() - 1; i >= 0; --i) {
            Edge ele = edges[i];

            // remove the edge temporarily
            edges.erase(edges.begin() + i);

            vector<vector<string>> scc = findSCC(graph);
            if ((int)scc.size() > initialScc) {
                _max = max(_max, (int)scc.size());
                candidates[scc.size()].push_back(ele);
            }

            // restore the edge
            edges.insert(edges.begin() + i, ele);
        }
    }
    if (_max == initialScc) return {};
    return candidates[_max];
}

/**
 * Suggests which two SCCs (groups of strongly connected airports) should be
 * connected to maximize reachability in the airline network.
 *
 * Computes all SCCs in the graph, sorts them by size in descending order,
 * and returns the two largest SCCs. If the network is already fully
 * connected (only one SCC), returns an empty vector.
 *
 */
vector<vector<string>> suggestHubConnection(adj& graph) {
    vector<vector<string>> scc = findSCC(graph);
    if (scc.size() <= 1) {
        return {};
    }

    // sort by descending order using the size of the connected components
    sort(scc.begin(), scc.end(),
         [](const vector<string> a, const vector<string> b) {
             return a.size() > b.size();
         });

    return {scc[0], scc[1]};  // return the first two largest connection
}

/**
 * Returns all airports reachable from a given start airport in a directed
 * graph.
 *
 * Performs a breadth-first search (BFS) starting from the `start` node and
 * collects all nodes that can be reached via one or more flights.
 *
 */
vector<string> reachableNodes(Node start, adj& graph) {
    vector<string> reachable;  // collects all reachable nodes

    queue<string> q;
    q.push(start.getCode());  // initializes the bfs with the starting node

    unordered_set<string>
        visited;  // keeps track of visited nodes to avoid cycle

    while (!q.empty()) {
        string current = q.front();
        q.pop();
        visited.insert(current);

        for (Edge& e : graph[current]) {
            // don't push to the queue if it is already visited
            if (visited.find(e.to) != visited.end()) continue;
            reachable.push_back(e.to);
            q.push(e.to);
        }
    }
    return reachable;
}
