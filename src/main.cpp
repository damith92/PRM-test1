

#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <tuple>
#include <algorithm>
#include <list>
#include <limits>
#include <set>
#include <tuple>
#include <map>

int INF = std::numeric_limits<int>::max();



// def of obstastacle

std::vector<int> obsX1{1, 5, 3,  1};
std::vector<int> obsY1{2, 4, 1,  2};

std::vector<int> obsX2{12, 13, 16,  12};
std::vector<int> obsY2{10, 9, 15,  10};

// data

int NODES = 500; 
float RADIUS = 5;
float startX = 5.0; 
float startY = 1.0; 
int startId = 0;
float goalX = 18.0; 
float goalY = 14.0; 
int goalId = NODES;



struct Node
{

    float x;
    float y;
    int id;
};

class Graph
{

private:
    int V;

    std::list<std::pair<int, int>> *adj;

public:
    std::vector<int> optimalNodes;
    Graph(int v) : V(v)
    {

        std::cout << "initialization of graph for A* ..." << std::endl;

        this->V = v;

        this->adj = new std::list<std::pair<int, int>>[this->V];
    }
    void addEdge(int vStart, int vEnd, int cost);
    void Astar(int vStart, int vGoal, std::vector<bool> visited, std::vector<float> &heuristic);
    void computeStarA(int vStart, int vGoal, std::vector<float> heuristic);
};

//---------------------------------------------------------------------------

bool onSegment(Node p, Node q, Node r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;

    return false;
}

int orientation(Node p, Node q, Node r)
{

    float val = (q.y - p.y) * (r.x - q.x) -
                (q.x - p.x) * (r.y - q.y);

    if (val == 0)
        return 0; // collinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool doIntersect(Node p1, Node q1, Node p2, Node q2)
{
    // Find the four orientations needed for general and
    // special cases
    float o1 = orientation(p1, q1, p2);
    float o2 = orientation(p1, q1, q2);

    float o3 = orientation(p2, q2, p1);
    float o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}

//-------------------------------------------------

void printGoalPath(std::vector<int> obsX1, std::vector<int> obsY1, std::vector<int> obsX2, std::vector<int> obsY2, std::vector<Node> nodes, std::vector<std::tuple<Node, Node>> knn_nodes, std::vector<int> optimalNodes)
{

    std::vector<float> optX;
    std::vector<float> optY;

    std::vector<float> nodeX;
    std::vector<float> nodeY;

    std::vector<float> x_knn;
    std::vector<float> y_knn;

    for (auto &ii : optimalNodes)
    {

        for (int jj = 0; jj < knn_nodes.size(); jj++)
        {
            Node a = std::get<0>(knn_nodes[jj]);
            if (ii == a.id)
            {

                optX.push_back(a.x);
                optY.push_back(a.y);
            }
        }
    }

    for (auto &ii : nodes)
    {
        nodeX.push_back(ii.x);
        nodeY.push_back(ii.y);
    }

    for (int ii = 0; ii < knn_nodes.size(); ii++)
    {

        Node a = std::get<0>(knn_nodes[ii]);
        Node b = std::get<1>(knn_nodes[ii]);

        x_knn.push_back(a.x);
        x_knn.push_back(b.x);
        y_knn.push_back(a.y);
        y_knn.push_back(b.y);
    }

    
}
//-------------------------------------------------

bool checkCollisonWithObs(Node a, Node b, std::vector<int> obsX1, std::vector<int> obsY1, std::vector<int> obsX2, std::vector<int> obsY2)
{

    bool collision = false;
    

    for (int ii = 0; ii < obsX1.size() - 1; ii++)
    {

        Node obs1, obs2;
        obs1.x = obsX1[ii];
        obs1.y = obsY1[ii];
        obs2.x = obsX1[ii + 1];
        obs2.y = obsY1[ii + 1];

        if (doIntersect(a, b, obs1, obs2))
        {

            collision = true;
        }
    }

    for (int ii = 0; ii < obsX2.size() - 1; ii++)
    {

        Node obs1, obs2;
        obs1.x = obsX2[ii];
        obs1.y = obsY2[ii];
        obs2.x = obsX2[ii + 1];
        obs2.y = obsY2[ii + 1];

        if (doIntersect(a, b, obs1, obs2))
        {

            collision = true;
        }
    }

    return collision;
}
//-------------------------------------------------

bool checkDistance(Node a, Node b)
{

    float dist = std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));

    return dist <= RADIUS ? true : false;
}

//------------------------------------------------

std::vector<std::tuple<Node, Node>> KNN(std::vector<Node> nodes)
{

    std::vector<std::tuple<Node, Node>> knn_nodes;

    for (int ii = 0; ii < nodes.size(); ii++)
    {

        for (int jj = 0; jj < nodes.size(); jj++)
        {

            if (ii != jj)
            {
                bool checkColl = checkCollisonWithObs(nodes[ii], nodes[jj], obsX1, obsY1, obsX2, obsY2);
                if (checkDistance(nodes[ii], nodes[jj]) && checkColl != true)
                {

                    knn_nodes.push_back({nodes[ii], nodes[jj]});
                }
            }
        }
    }

    return knn_nodes;
}

//-------------------------------------------------

std::vector<Node> nodeObsExclusion(std::vector<int> obsX1, std::vector<int> obsY1, std::vector<int> obsX2, std::vector<int> obsY2, std::vector<Node> vecNodes)
{

    std::vector<Node> vecNodesEx;

    std::sort(obsX1.begin(), obsX1.end());
    std::sort(obsY1.begin(), obsY1.end());
    std::sort(obsX2.begin(), obsX2.end());
    std::sort(obsY2.begin(), obsY2.end());

    float x_min_obs1 = obsX1[0];
    float x_max_obs1 = obsX1[obsX1.size() - 1];
    float y_min_obs1 = obsY1[0];
    float y_max_obs1 = obsY1[obsY1.size() - 1];

    float x_min_obs2 = obsX2[0];
    float x_max_obs2 = obsX2[obsX2.size() - 1];
    float y_min_obs2 = obsY2[0];
    float y_max_obs2 = obsY2[obsY2.size() - 1];

    for (auto &ii : vecNodes)
    {
        bool col1 = false;
        bool col2 = false;

        if (ii.x > x_min_obs1 && ii.x < x_max_obs1 && ii.y > y_min_obs1 && ii.y < y_max_obs1)
        {
            col1 = true;
        }

        if (ii.x > x_min_obs2 && ii.x < x_max_obs2 && ii.y > y_min_obs2 && ii.y < y_max_obs2)
        {

            col2 = true;
        }

        if (col1 == false && col2 == false)
        {

            vecNodesEx.push_back(ii);
        }
    }

    return vecNodesEx;
}

//-------------------------------------------------

std::vector<Node> generateNodes()
{

    std::vector<Node> vecNodes;
    std::vector<Node> vecNodesEx;
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int ii = 1; ii < NODES + 1; ii++)
    {

        std::uniform_int_distribution<> dist(0, 200);

        Node node{node.x = dist(gen) / 10.0, node.y = dist(gen) / 10.0, node.id = ii};
        vecNodes.push_back(node);
    }
    Node nodeS{nodeS.x = startX, nodeS.y = startY, nodeS.id = startId};
    Node nodeG{nodeG.x = goalX, nodeG.y = goalY, nodeG.id = goalId};
    vecNodes.push_back(nodeS);
    vecNodes.push_back(nodeG);
    return nodeObsExclusion(obsX1, obsY1, obsX2, obsY2, vecNodes);
}

//-------------------------------------------------------------------------------

void Graph::addEdge(int vStart, int vEnd, int cost)
{

    this->adj[vStart].push_back(std::make_pair(cost, vEnd));
}

//-------------------------------------------------------------------------------
/*
    g = min cost from start vertex to this vertex (cost is changing depending which vertexes have benn visited)
    h = heuristic (eg Manhattan distance) - constance
    (FX) f = g + h
    next step: min f

    */

void Graph::Astar(int vStart, int vGoal, std::vector<bool> visited, std::vector<float> &heuristic)
{

    std::vector<std::tuple<int, int, int>> path;
    std::vector<float> functionFX(V, INF);

    std::set<std::pair<float, int>> AStar_set;

    functionFX[vStart] = 0 + heuristic[vStart];

    AStar_set.insert(std::make_pair(functionFX[vStart], vStart));

    while ((*(AStar_set.begin())).second != vGoal)
    {

        std::pair<float, int> nodeMin = *(AStar_set.begin());

        int nodeGraph = nodeMin.second;

        AStar_set.erase(AStar_set.begin());

        visited[nodeGraph] = true;

        // traverse list for certein vertex

        for (auto i = adj[nodeMin.second].begin(); i != adj[nodeMin.second].end(); i++)
        {

            int nodeGraph_i = (*i).second;
            int nodeGraph_i_functionFX = (*i).first + heuristic[(*i).second];

            // check the cost - functionFX for each neighbors of current vertex
            if (visited[nodeGraph_i] != true)
            {

                if (functionFX[nodeGraph_i] > nodeGraph_i_functionFX)
                {
                    // Remove the current distance if it is in the set
                    if (functionFX[nodeGraph_i] != INF)
                    {
                        AStar_set.erase(AStar_set.find(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i)));
                    }

                    // Update the distance
                    functionFX[nodeGraph_i] = nodeGraph_i_functionFX;
                    AStar_set.insert(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i));

                    path.push_back(std::make_tuple(functionFX[nodeGraph_i], nodeGraph_i, nodeGraph));
                }
            }
        }
    }

    //---------------------------------------------------------------------

    std::multiset<std::tuple<int, int>> init_mSet;
    init_mSet.insert(std::make_tuple(0, 0));
    std::vector<std::multiset<std::tuple<int, int>>> routePath(V, init_mSet);

    for (int pathV = 1; pathV < V; pathV++)
    {

        std::multiset<std::tuple<int, int>> to_routePath;

        for (auto &ii : path)
        {

            int vertexV = std::get<1>(ii);

            if (pathV == vertexV)
            {

                to_routePath.insert(std::make_tuple(std::get<0>(ii), std::get<2>(ii))); //  Path_FX : Xx_xX, previous :Xx_xX
            }
        }

        routePath[pathV] = to_routePath;
    }

    //---------------------------------------------------------------------

    int previous = vGoal;
    std::vector<int> optimalPath;
    optimalPath.push_back(vGoal);

    while (previous != 0)
    {

        std::set<std::tuple<int, int>> minFx;

        for (auto &ii : routePath[previous])
        {

            minFx.insert(std::make_tuple(std::get<0>(ii), std::get<1>(ii)));
        }

        auto it = minFx.begin();
        previous = std::get<1>(*it);

        int min_path_i = std::get<0>(*it);

        optimalPath.push_back(previous);
    }

    optimalNodes = optimalPath;
    //======= PRINT OPTIMAL PATH ===========

    std::cout << "Optimal A* path for given graph : " << std::endl;
    for (auto &ii : optimalPath)
    {

        std::cout << ii << "--";
    }

    std::cout << "\n";

    //======================================
}

//-------------------------------------------------------------------------------

void Graph::computeStarA(int vStart, int vGoal, std::vector<float> heuristic)
{

    std::vector<bool> visited(this->V, false);

    Graph::Astar(vStart, vGoal, visited, heuristic);
}

//-------------------------------------------------------------------------------

float measureNodeDistance(Node a, Node b)
{

    return std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));
}

//--------------------------------------------------------------------------------

int main()
{
    std::vector<Node> nodes = generateNodes();
    std::vector<std::tuple<Node, Node>> knn_nodes = KNN(nodes);

    Node G_h{G_h.x = goalX, G_h.y = goalY, G_h.id = goalId};

    int astar_nodes = knn_nodes.size();
    std::vector<float> heuristic;

    Graph g(astar_nodes);

    for (int ii = 0; ii < knn_nodes.size(); ii++)
    {

        Node a = std::get<0>(knn_nodes[ii]);
        Node b = std::get<1>(knn_nodes[ii]);

        float dist = measureNodeDistance(a, b);
        heuristic.push_back(measureNodeDistance(a, G_h));

        g.addEdge(a.id, b.id, dist);
    }

    g.computeStarA(startId, goalId, heuristic);

    for (auto &ii : g.optimalNodes)
    {

        std::cout << ii << " , ";
    }

    printGoalPath(obsX1, obsY1, obsX2, obsY2, nodes, knn_nodes, g.optimalNodes);
}
