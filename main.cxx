#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <limits>

// -------------------------------------------------------------------------
struct Point
{
  float X, Y, Z;
  float distanceTo(const Point &b) const
  {
    float x = X - b.X;
    float y = Y - b.Y;
    float z = Z - b.Z;
    return std::sqrt((x * x) + (y * y) + (z * z));
  }
};

// -------------------------------------------------------------------------
struct Edge
{
  int start, end;
  float cost;
};

struct Graph
{
  std::vector<Point> vertices;
  std::vector<std::vector<Edge>> adjList;

  void AddVertex(const Point &p)
  {
    vertices.push_back(p);
    adjList.push_back({});
  }

  void AddEdge(int start, int end, float cost)
  {
    adjList[start].push_back({start, end, cost});
  }

  std::vector<int> Dijkstra(int start, int end)
  {
    std::vector<float> dist(vertices.size(), std::numeric_limits<float>::infinity());
    std::vector<int> prev(vertices.size(), -1);
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> pq;

    dist[start] = 0.0;
    pq.push({0.0, start});

    while (!pq.empty())
    {
      int u = pq.top().second;
      pq.pop();

      if (u == end)
        break;

      for (const auto &edge : adjList[u])
      {
        int v = edge.end;
        float weight = edge.cost;
        float alt = dist[u] + weight;

        if (alt < dist[v])
        {
          dist[v] = alt;
          prev[v] = u;
          pq.push({alt, v});
        }
      }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = prev[at])
    {
      path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    if (path.size() == 1 && path[0] != start)
    {
      path.clear();
    }

    return path;
  }
};

// -------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0] << " input_mesh start end" << std::endl;
    return 1;
  }

  long start_id = std::atoi(argv[2]);
  long end_id = std::atoi(argv[3]);

  Graph g;

  // Load file in a buffer
  std::ifstream in_mesh_stream(argv[1], std::ifstream::binary);
  if (!in_mesh_stream)
  {
    std::cerr << "Error reading \"" << argv[1] << "\"" << std::endl;
    return 1;
  }

  in_mesh_stream.seekg(0, in_mesh_stream.end);
  unsigned long in_mesh_file_length = in_mesh_stream.tellg();
  in_mesh_stream.seekg(0, in_mesh_stream.beg);
  char *in_mesh_file_buffer = new char[in_mesh_file_length];
  in_mesh_stream.read(in_mesh_file_buffer, in_mesh_file_length);
  in_mesh_stream.close();
  std::istringstream in_mesh(in_mesh_file_buffer);

  // Read vertices
  long nPoints;
  in_mesh >> nPoints;
  for (long pId = 0; pId < nPoints; pId++)
  {
    Point pnt;
    in_mesh >> pnt.X >> pnt.Y >> pnt.Z;
    g.AddVertex(pnt);
  }

  // Read edges
  long nEdges;
  in_mesh >> nEdges;
  for (long eId = 0; eId < nEdges; eId++)
  {
    long start, end;
    in_mesh >> start >> end;

    float cost = g.vertices[start].distanceTo(g.vertices[end]);
    g.AddEdge(start, end, cost);
    g.AddEdge(end, start, cost);
  }
  delete[] in_mesh_file_buffer;

  if (start_id < 0 || start_id >= g.vertices.size() || end_id < 0 || end_id >= g.vertices.size())
  {
    std::cerr << "Invalid path endpoints." << std::endl;
    return 1;
  }

  std::vector<int> path = g.Dijkstra(start_id, end_id);
  std::cout << path.size() << std::endl;
  for (unsigned int i = 0; i < path.size(); ++i)
  {
    std::cout << g.vertices[path[i]].X << " " << g.vertices[path[i]].Y << " " << g.vertices[path[i]].Z << std::endl;
  }

  return 0;
}
