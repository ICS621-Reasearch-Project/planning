package project_621_finals;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;
import project_621_finals.Graph_Object.Graph;

public class SteinerTreeDriver {

    // Graph Generating
    public static class GraphGenerator {
        public static Graph generateGraph(int n, int m) {
            Graph g = new Graph(n);
            Random rand = new Random();

            // Spanning Tree Generation
            for (int i = 1; i < n; i++) {
                int u = rand.nextInt(i);
                int weight = 1;
                g.addEdge(u, i, weight);
            }

            // Add extra edges
            int extraEdges = m - (n - 1);
            for (int i = 0; i < extraEdges; i++) {
                int u = rand.nextInt(n);
                int v = rand.nextInt(n);
                if (u != v) {
                    int weight = 1;
                    g.addEdge(u, v, weight);
                } else {
                    i--;
                }
            }
            return g;
        }
    }

    // Floyd-Warshall Algorithmns for all shortest paths
    private static int[][] allShortestPaths(Graph G) {
        int n = G.n;
        int[][] dist = new int[n][n];
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
                    if (dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }
        return dist;
    }

    // Min-length Steiner Trees
    private static List<int[]> steinerTree(Graph G, int[][] dist, Set<Integer> Tset) {
        // Construct MST on metric closure approach
        List<Integer> Tnodes = new ArrayList<>(Tset);
        int k = Tnodes.size();
        int[] key = new int[k];
        int[] parent = new int[k];
        boolean[] mstSet = new boolean[k];
        Arrays.fill(key, Integer.MAX_VALUE/2);
        key[0] = 0;
        Arrays.fill(parent, -1);

        for (int count = 0; count < k - 1; count++) {
            int u = -1; int minVal = Integer.MAX_VALUE/2;
            for (int i = 0; i < k; i++) {
                if (!mstSet[i] && key[i]<minVal) {
                    minVal = key[i]; u = i;
                }
            }
            mstSet[u] = true;
            for (int v = 0; v < k; v++) {
                if (!mstSet[v] && dist[Tnodes.get(u)][Tnodes.get(v)] < key[v]) {
                    key[v] = dist[Tnodes.get(u)][Tnodes.get(v)];
                    parent[v] = u;
                }
            }
        }

        List<int[]> Cedges = new ArrayList<>();

        // Currently not expanding edges fully (placeholder)
        return Cedges;
    }

    // Round Robin Like Partiotions to ensure evenly distributed elements
    private static List<Set<Integer>> partitionSet(Set<Integer> S_rho, int k) {
        List<Integer> list = new ArrayList<>(S_rho);
        List<Set<Integer>> parts = new ArrayList<>();
        for (int i = 0; i < k; i++) parts.add(new HashSet<>());
        for (int i = 0; i < list.size(); i++) {
            parts.get(i % k).add(list.get(i));
        }
        return parts;
    }

    // main test:
    public static void main(String[] args) throws IOException {
        int graphSize = 2500;
        int edges = 7500;
        int[] agentCounts = {1,2,4,8};

        Graph g = GraphGenerator.generateGraph(graphSize, edges);
        int[][] dist = allShortestPaths(g);
        // Prepare results
        List<String> results = new ArrayList<>();

        Set<Integer> terminals = new HashSet<>();
        Random rand = new Random();
        // Testing number of terminals
        int terminalCount = 1000;
        while (terminals.size() < terminalCount) {
            terminals.add(rand.nextInt(graphSize));
        }

        File file = new File("steiner.txt");
        boolean fileExists = file.exists() && file.length() > 0;

        // If you only want a single header:
        try (PrintWriter out = new PrintWriter(new FileWriter(file, true))) {
            if (!fileExists) {
                out.println("Terminals: " + terminalCount);
            }
        }

        // Now test splitting these terminals among k agents:
        System.out.println("Terminals: " + terminalCount);
        for (int k : agentCounts) {
            // Clear results each time so we don't append duplicates
            results.clear();

            // Partition terminals:
            List<Set<Integer>> parts = partitionSet(terminals, k);
            // Start Time Measurement
            long start = System.currentTimeMillis();
            int totalEdges = 0;
            for (Set<Integer> part : parts) {
                Set<Integer> Tset = new HashSet<>(part);
                int root = 0;
                Tset.add(root);
                List<int[]> Cedges = steinerTree(g, dist, Tset);
                totalEdges += Cedges.size();
            }
            // End Time Measurement
            long end = System.currentTimeMillis();
            double timeSec = (end - start)/1000.0;

            System.out.println(k + " agents: time=" + timeSec);
            System.out.println(k + " agents: terminals_per_agent ~ " + (terminalCount / k));
            results.add(k + " agents: time=" + timeSec + "sec, approx_total_steiner_edges=" + totalEdges);
            results.add(k + " agents: terminals_per_agent ~ " + (terminalCount / k));

            try (PrintWriter out = new PrintWriter(new FileWriter(file, true))) {
                for (String line : results) {
                    out.println(line);
                }
            }

            System.out.println("Results appended to steiner.txt");
        }
        
        try (PrintWriter out = new PrintWriter(new FileWriter(file, true))) {
          out.println();
      }
    }
}
