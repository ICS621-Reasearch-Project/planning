package project_621_finals;

import java.util.ArrayList;
import java.util.List;

public class Graph_Object{
  
  // Graph Object
  static class Graph {
    int n;
    List<Edge>[] adj;

    // Edge Object
    static class Edge {
        int to;
        int length;
        Edge(int to, int length) {
            this.to = to;
            this.length = length;
        }
    }

    @SuppressWarnings("unchecked")
    // Graph constructor with edges amount as parameter
    Graph(int n) {
        this.n = n;
        adj = new ArrayList[n];
        // Adding Edge Arraylist
        for (int i = 0; i < n; i++) {
            adj[i] = new ArrayList<>();
        }
    }

    // Edge adding helper method
    void addEdge(int u, int v, int length) {
        // Undirected edge
        adj[u].add(new Edge(v, length));
        adj[v].add(new Edge(u, length));
    }
}

}
