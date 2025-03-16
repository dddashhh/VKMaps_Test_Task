# Shortest Path Finder

![Graph Visualization Demo](./Example.png)

## Overview
This program implements Dijkstra, BFS, Bellman-Ford, and Floyd-Warshall algorithms to find the shortest path from a specified source vertex to all other vertices in an undirected graph. The application reads the graph structure from a file and outputs the shortest distance to each vertex.

## Features
- Reads the graph structure from a file
- Calculates shortest paths using different algorithms
- Supports undirected graphs with non-negative edge weights
- Displays the distances from the source vertex to all other vertices
- Visualizes the graph and the selected route

## File Format
The input file must follow this fixed structure:
```
n       // Number of vertices
m       // Number of edges
u1 v1   // Edge 1: from vertex u1 to vertex v1
u2 v2   // Edge 2: from vertex u2 to vertex v2
...
um vm   // Edge m: from vertex um to vertex vm
s       // Source vertex
```

Example file (graph.txt):
```
5       // 5 vertices (0-4)
4       // 4 edges
0 1     // Edge from vertex 0 to vertex 1
0 4     // Edge from vertex 0 to vertex 4
1 2     // Edge from vertex 1 to vertex 2
1 3     // Edge from vertex 1 to vertex 3
4       // Source vertex = 4
```

Notes:
- Vertex numbering starts from 0
- All edges have a weight of 1
- Each edge is automatically considered in both directions (undirected graph)

## Example Output
For the example input file above, if the source vertex is 4, the output might be:
```
1
2
3
3
0
```
Meaning:
- Distance from vertex 4 to vertex 0 is 1
- Distance from vertex 4 to vertex 1 is 2
- Distance from vertex 4 to vertex 2 is 3
- Distance from vertex 4 to vertex 3 is 3
- Distance from vertex 4 to vertex 4 (itself) is 0
  
## Contact Information

- **Developer:** Daria Karaseva
- **Email:** dar1a.karaseva@yandex.ru
- **Telegram:** @dashkar
