package project_621_finals;

import java.util.*;

import project_621_finals.Graph_Object.Graph;

public class FullInfoX_Multi {
 
    // FullInfoXAgent Object
    public static class FullInfoXAgent extends Thread {
        // Thread Control Master
        private FullInfoX_Multi master;
        private Set<Integer> subset;
        private int currentR;
        private int lastVisitedNode = -1;
        
        // FullInfoXAgent Constructor
        FullInfoXAgent(FullInfoX_Multi master, Set<Integer> subset, int currentR) {
            this.master = master;
            this.subset = subset;
            this.currentR = currentR;
        }

        @Override
        // Run Method for FullInfoXAgent
        public void run() {
            // Ensure termination when empty of node or goal found in another thread
            if (subset.isEmpty() || master.goalFound()) return;

            Set<Integer> terminalSet = new HashSet<>(subset);
            terminalSet.add(currentR);
            List<int[]> steinerEdges = master.steinerTree(terminalSet);

            // Arbitrary Node Transitions
            int r_rho = subset.iterator().next();
            travelPathFromTo(currentR, r_rho);
            // Incase found in transitions
            if (master.goalFound()) return;
            // Building Adjacency list edges
            Map<Integer,List<Integer>> treeAdj = master.buildTreeAdj(steinerEdges);
            // Building Euler tour
            List<Integer> eulerTour = master.buildEulerTour(treeAdj, r_rho);
            for (int v : eulerTour) {
                // Ensure synchronizations when goal found in other threads
                if (master.goalFound()) break; 
                // Thread Safe Modification 
                synchronized(master) { 
                    if (!master.visited[v]) {
                        master.visited[v] = true;
                        master.traveledPath.add(v);
                        // Direct check for goal using a hard-coded goalNodeId
                        if (v == master.goalNodeId) {
                            master.goal = v;
                            break;
                        }
                    } else {
                        master.traveledPath.add(v);
                    }
                    lastVisitedNode = v;
                }
            }
        }

        // Current node to Arbitrary Node Transitions 
        private void travelPathFromTo(int u, int v) {
            // Compute Shorest Path first
            int[] par = master.shortestPath(u,v);
            List<Integer> path = reconstructPath(u,v,par);
            for (int x : path) {
                if (x != u) {
                    synchronized(master) {
                        master.traveledPath.add(x);
                        if (!master.visited[x]) {
                            master.visited[x] = true;
                            // Direct check for goal using a hard-coded goalNodeId
                            if (x == master.goalNodeId) {
                                master.goal = x;
                                return;
                            }
                        }
                    }
                }
                if (master.goalFound()) return;
            }
        }

        private List<Integer> reconstructPath(int u,int v,int[] par){
            List<Integer> p=new ArrayList<>();
            for(int cur=v; cur!=-1; cur=par[cur]) {
              p.add(cur);
            }
            Collections.reverse(p);
            return p;
        }

        public int getLastNodeVisited() { return lastVisitedNode; }
    }

    private Graph G;
    private int[] f;
    private int start;
    // Testing Purpose only
    private int goalNodeId = 58; 
    int goal = -1; 

    int[][] dist;
    boolean[] visited;
    List<Integer> traveledPath = Collections.synchronizedList(new ArrayList<>());
    private int k;
    private FullInfoXAgent[] agents;

    // FullInfoX_Multi Constructor
    public FullInfoX_Multi(Graph G, int[] f, int start, int k) {
        this.G = G; 
        this.f = f; 
        this.start = start; 
        this.k = k;
        this.visited = new boolean[G.n];
        allShortestPath();
        // List of agents
        agents = new FullInfoXAgent[k];
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
        for (int kk = 0; kk < n; kk++) {
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    // Relaxation Step
                    if (dist[i][j] > dist[i][kk] + dist[kk][j]) {
                        dist[i][j] = dist[i][kk] + dist[kk][j];
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

    // Multi Agent Run Method
    public void run() throws InterruptedException {
      
        Set<Integer> allVisited = new HashSet<>();
        int p = 0;
        int currentR = start;

        while (!goalFound()) {
            // Current round of node to be visited
            Set<Integer> roundNode = new HashSet<>();
            // 2^p Threshold
            int threshold = 1 << p;
            for (int v = 0; v < G.n; v++) {
                // Check If Visited In All Previous Round
                if (!allVisited.contains(v) && phi(v) < threshold) {
                  roundNode.add(v);
                }
            }

            if (!roundNode.isEmpty()) {
                List<Set<Integer>> partitions = partitionSet(roundNode, k);
                // Assign task to each agents 
                for (int i = 0; i < k; i++) {
                    agents[i] = new FullInfoXAgent(this, partitions.get(i), currentR);
                    agents[i].start();
                }
                // Wait for all agent to be completed
                for (int i = 0; i < k; i++) {
                    agents[i].join();
                }

                if (goalFound()) break;
                for (Set<Integer> part : partitions) {
                  allVisited.addAll(part);
                }
            }
            if (goalFound()) break;
            p++;
            if (p > G.n) {
                break;
            }
        }
    }

    // Round Robin Like partitions to ensure evenly distributed elements
    private List<Set<Integer>> partitionSet(Set<Integer> S_rho, int k) {
        List<Integer> list = new ArrayList<>(S_rho);
        List<Set<Integer>> parts = new ArrayList<>();
        for (int i = 0; i < k; i++) parts.add(new HashSet<>());
        for (int i = 0; i < list.size(); i++) {
            parts.get(i % k).add(list.get(i));
        }
        return parts;
    }

    // Return With ThreadSafe methods
    public synchronized boolean goalFound() {
        return goal != -1;
    }

    // Dijkstra Algorithmn for Shortest Path
    private int[] shortestPath(int start, int end) {
        int n = G.n;
        int[] distArr = new int[n];
        int[] par = new int[n];
        Arrays.fill(distArr, Integer.MAX_VALUE/2);
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

    // Min-Lenght Steiner Tree
    private List<int[]> steinerTree(Set<Integer> Tset) {
        List<Integer> Tnodes = new ArrayList<>(Tset);
        int k = Tnodes.size();
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
                    minVal = key[i]; u = i;
                }
            }
            // Set terminal as part of mst
            mstSet[u] = true;
            for (int v = 0; v < k; v++) {
                // To avoid more expensive opertaion, we use original computed path
                // If exist an shorter path then existing key[v]
                if (!mstSet[v] && dist[Tnodes.get(u)][Tnodes.get(v)]<key[v]) {
                    key[v] = dist[Tnodes.get(u)][Tnodes.get(v)];
                    parent[v] = u;
                }
            }
        }

        // Expand edges of MST back into graphs
        List<int[]> Cedges = new ArrayList<>();
        for (int i = 1; i < k; i++) {
            int p = parent[i];
            int u = Tnodes.get(i), w = Tnodes.get(p);
            int[] par = shortestPath(u,w);
            List<Integer> path = reconstructPath(u,w,par);
            for (int idx = 0; idx < path.size()-1; idx++) {
                Cedges.add(new int[]{path.get(idx), path.get(idx+1)});
            }
        }

        return Cedges;
    }

    // Path Restoration Method
    private List<Integer> reconstructPath(int u,int v,int[] par){
        List<Integer> p=new ArrayList<>();
        for(int cur=v;cur!=-1;cur=par[cur]) p.add(cur);
        Collections.reverse(p);
        return p;
    }

    // Building Adjacency list edges for all nodes
    Map<Integer,List<Integer>> buildTreeAdj(List<int[]> edges) {
        Map<Integer,List<Integer>> adj = new HashMap<>();
        for (int[] e : edges) {
            adj.computeIfAbsent(e[0], x->new ArrayList<>()).add(e[1]);
            adj.computeIfAbsent(e[1], x->new ArrayList<>()).add(e[0]);
        }
        return adj;
    }

    // Euler tour on a tree via DFS
    List<Integer> buildEulerTour(Map<Integer,List<Integer>> treeAdj, int root) {
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
