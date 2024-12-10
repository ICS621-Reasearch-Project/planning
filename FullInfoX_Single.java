package project_621_finals;

import java.util.*;

import project_621_finals.Graph_Object.Graph;


// FullInfoX in book
public class FullInfoX_Single {

  // FullInfoX object along with its methods and instances
  public static class FullInfoX {
    private Graph G;
    // predictions array, represents f(v)
    private int[] f;
    private int start;
    private int goal = -1; 
    // For Testing Purposes Only 
    private int goalNodeId = 58;

    // Storing shortest distance between u and v, dist[u][v]
    private int[][] dist;
    
    // To be called in driver files
    public int[][] getDist() {
      return dist;
   }

    // Track visited nodes
    private boolean[] visited;
    private List<Integer> traveledPath = new ArrayList<>();

    // FullInfoX Constructor 
    public FullInfoX(Graph G, int[] f, int start) {
      this.G = G;
      this.f = f;
      this.start = start;
      this.visited = new boolean[G.n];
      // Precompute shortest paths
      allShortestPath();
    }
    
    // Floyd Warshall algorithm for all shortest paths
    // Unimportant for FullInfoX analysis
    // Used for preparing graphs and calculating error
    private void allShortestPath() {
      int n = G.n;
      dist = new int[n][n];
      for (int i = 0; i < n; i++) {
        Arrays.fill(dist[i], Integer.MAX_VALUE / 2);
        dist[i][i] = 0;
        for (Graph.Edge e : G.adj[i]) {
          dist[i][e.to] = Math.min(dist[i][e.to], e.length);
        }
      }
      for (int k = 0; k < n; k++) {
        for (int i = 0; i < n; i++) {
          for (int j = 0; j < n; j++) {
            // Relaxation Step
            if (dist[i][j] > dist[i][k] + dist[k][j]) {
              dist[i][j] = dist[i][k] + dist[k][j];
            }
          }
        }
      }
    }

    // Compute error prediction, ϕ(v)
    private int phi(int v) {
      // Cumulative Error Count
      int count = 0;
      for (int u = 0; u < G.n; u++) {
        if (f[u] != dist[u][v]) {
          count++;
        }
      }
      return count;
    }

    // FullInfoX Algorithmns, to be called by FullInfoX.run()
    public void run() {
      
      List<Set<Integer>> Ssets = new ArrayList<>();
      // All Previous Visited Set
      Set<Integer> allVisited = new HashSet<>();
      // Threshold
      int p = 0;
      int currentR = start;
      
     
      while (goal == -1) {
        // Current round of node to be visited
        Set<Integer> roundNode = new HashSet<>();
        // 2^p Threshold
        int threshold = 1 << p;
        for (int v = 0; v < G.n; v++) {
          // Check If Visited In All Previous Round
          if (!allVisited.contains(v)) {
            if (phi(v) < threshold) {
              roundNode.add(v);
            }
          }
        }

        // Iterate through current set 
        if (!roundNode.isEmpty()) {
        
          // Steiner Tree Construction
          Set<Integer> terminalSet = new HashSet<>(roundNode);
          terminalSet.add(currentR);
          List<int[]> steinerEdges = steinerTree(terminalSet);

          // Arbitrary Node Transitions
          int r_rho = roundNode.iterator().next();
          travelPathFromTo(currentR, r_rho);

          // Euler tour of min steiner tree
          // Build an adjacency map for quick euler tour look up
          Map<Integer, List<Integer>> treeAdj = buildTreeAdj(steinerEdges);
          List<Integer> eulerTour = buildEulerTour(treeAdj, r_rho);
          for (int v : eulerTour) {
            if (!visited[v]) {
              visited[v] = true;
              traveledPath.add(v);
              // Direct check for goal
              if (v == goalNodeId) {
                goal = v;
                break;
              }
            } else {
              traveledPath.add(v);
            }
          }
          // Return to start
          currentR = r_rho;
          Ssets.add(roundNode);
          // Add all visited in this round
          allVisited.addAll(roundNode);
        }

        // Termination Check
        if (goal != -1) break;
        p++;
        if (p > G.n) {
          break;
        }
      }
    }

    // Current node to Arbitrary Node Transitions 
    private void travelPathFromTo(int u, int v) {
      // Compute Shorest Path first
      int[] par = shortestPath(u, v);
      List<Integer> path = new ArrayList<>();
      for (int i = v; i != -1; i = par[i]) {
        path.add(i);
      }
      Collections.reverse(path);
      
      // Checking for goal node if it lies on transitions path
      for (int x : path) {
        if (x != u) {
          traveledPath.add(x);
          if (!visited[x]) {
            visited[x] = true;
            // Direct goal check
            if (x == goalNodeId) {
              goal = x;
              return;
            }
         }
       }
     }
    }

    // Dijkstra Algorithmn for Shortest Path 
    private int[] shortestPath(int start, int end) {
      int n = G.n;
      int[] distArr = new int[n];
      int[] par = new int[n];
      Arrays.fill(distArr, Integer.MAX_VALUE / 2);
      Arrays.fill(par, -1);
      distArr[start] = 0;
      PriorityQueue<int[]> pq = new PriorityQueue<>(Comparator.comparingInt(x->x[1]));
      pq.add(new int[]{start,0});
      boolean[] visitedDij = new boolean[n];

      while(!pq.isEmpty()) {
        int[] top = pq.poll();
        int u = top[0]; int d = top[1];
        if (visitedDij[u]) continue;
        visitedDij[u] = true;
        if (u == end) break;
        for (Graph.Edge e : G.adj[u]) {
          if (distArr[e.to] > d + e.length) {
            distArr[e.to] = d + e.length;
            par[e.to] = u;
            pq.add(new int[]{e.to, distArr[e.to]});
          }
        }
      }
      return par;
    }

    // Min Length Steiner Tree
    private List<int[]> steinerTree(Set<Integer> terminalSet) {
      List<Integer> terminalNodes = new ArrayList<>(terminalSet);
      int k = terminalNodes.size();
      int[] key = new int[k];
      int[] parent = new int[k];
      boolean[] mstSet = new boolean[k];
      Arrays.fill(key, Integer.MAX_VALUE/2);
      key[0] = 0;
      Arrays.fill(parent, -1);

      // Prims algorithm for MST
      for (int count = 0; count < k-1; count++) {
        // Picking Terminal
        int u = -1; 
        int minVal = Integer.MAX_VALUE/2;
        for (int i = 0; i < k; i++) {
          if (!mstSet[i] && key[i]<minVal) {
            // Minimum Known Distance 
            minVal = key[i]; 
            u = i;
          }
        }
        // Set terminal as part of mst
        mstSet[u] = true;
        for (int v = 0; v < k; v++) {
          // To avoid more expensive opertaion, we use original computed path
          // If exist an shorter path then existing key[v]
          if (!mstSet[v] && dist[terminalNodes.get(u)][terminalNodes.get(v)]<key[v]) {
            key[v] = dist[terminalNodes.get(u)][terminalNodes.get(v)];
            parent[v] = u;
          }
        }
      }

      // Expand edges of MST back into graphs
      List<int[]> Cedges = new ArrayList<>();
      for (int i = 1; i < k; i++) {
        
        int p = parent[i];
        int u = terminalNodes.get(i);
        int w = terminalNodes.get(p);
        int[] par = shortestPath(u,w);
        List<Integer> path = new ArrayList<>();
        
        // Path from w to u
        for (int cur = w; cur != -1; cur = par[cur]) {
          if (cur==u) {
            path.add(cur);
            break;
          }
          path.add(cur);
        }
        Collections.reverse(path);
        path.clear();
        for (int cur = w; cur != -1; cur = par[cur]) {
          path.add(cur);
          if (cur == u) break;
        }
        // Reverse back to order, path from u to w
        Collections.reverse(path);

        // Convert path to edges
        for (int idx = 0; idx < path.size()-1; idx++) {
          Cedges.add(new int[]{path.get(idx), path.get(idx+1)});
        }
      }
      
      return Cedges;
    }

    // Building Adjacency list edges for all nodes
    private Map<Integer,List<Integer>> buildTreeAdj(List<int[]> edges) {
      Map<Integer,List<Integer>> adj = new HashMap<>();
      for (int[] e : edges) {
        adj.computeIfAbsent(e[0], x->new ArrayList<>()).add(e[1]);
        adj.computeIfAbsent(e[1], x->new ArrayList<>()).add(e[0]);
      }
      return adj;
    }

    // Euler tour on a tree via DFS
    private List<Integer> buildEulerTour(Map<Integer,List<Integer>> treeAdj, int root) {
      for (int i = 0; i < G.n; i++) {
        treeAdj.putIfAbsent(i, new ArrayList<>());
      }
      List<Integer> tour = new ArrayList<>();
      boolean[] vis = new boolean[G.n];
      dfsEuler(root, -1, treeAdj, vis, tour);
      tour.add(root);
      return tour;
    }

    // DFS helper function for euler tour 
    private void dfsEuler(int u, int p, Map<Integer,List<Integer>> adj, boolean[] vis, List<Integer> tour) {
      vis[u] = true;
      tour.add(u);
      for (int w : adj.get(u)) {
        if (w == p) continue;
        if (!vis[w]) {
          dfsEuler(w,u,adj,vis,tour);
          tour.add(u);
        }
      }
    }
  }
}
