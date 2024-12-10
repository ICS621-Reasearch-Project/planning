package project_621_finals;

import java.io.*;
import java.util.*;
import project_621_finals.FullInfoX_Single.FullInfoX;
import project_621_finals.Graph_Object.Graph;

public class Driver {

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

  // Randomized Prediction
  public static class PredictionGenerator {
      public static int[] generatePredictions(int n, int[][] dist, int goalNodeId) {
          Random rand = new Random();
          int[] f = new int[n];
          for (int v = 0; v < n; v++) {
              int trueDist = dist[v][goalNodeId];
              // small noise [-2,2]
              int noise = rand.nextInt(5) - 2;
              // bigger error [-10,10]
              if (rand.nextDouble() < 0.1) {
                  noise = rand.nextInt(21) - 10;
              }

              int predicted = trueDist + noise;
              if (predicted < 0) predicted = 0;
              f[v] = predicted;
          }
          return f;
      }
  }

  // Driver Class
  public static void main(String[] args) throws InterruptedException, IOException {
      // Parameters
      int graphSize = 1500;
      int edges = 5000;
      int[] numAgents = {1, 2, 4, 8};

      Graph g = GraphGenerator.generateGraph(graphSize, edges);

      // Compute dist using dummy FullInfoX
      int[] dummyF = new int[graphSize];
      Arrays.fill(dummyF, 0);
      FullInfoX dummyFullInfo = new FullInfoX(g, dummyF, 0);
      int[][] dist = dummyFullInfo.getDist();

      int goalNodeId = 58;
      int[] f = PredictionGenerator.generatePredictions(graphSize, dist, goalNodeId);

      // Single-agent run
      FullInfoX singleAgent = new FullInfoX(g, f, 0);
      long startTime = System.currentTimeMillis();
      singleAgent.run();
      long endTime = System.currentTimeMillis();
      long singleTime = endTime - startTime;
      System.out.println("Single-Agent FullInfoX: " + (singleTime / 1000.0) + " sec");

      // Prepare results
      List<String> results = new ArrayList<>();
      results.add("FullInfoX," + graphSize + "," + edges + ",1," + (singleTime / 1000.0));

      // Multi-agent runs
      for (int k : numAgents) {
          FullInfoX_Multi multiAgent = new FullInfoX_Multi(g, f, 0, k);
          startTime = System.currentTimeMillis();
          multiAgent.run();
          endTime = System.currentTimeMillis();
          long multiTime = endTime - startTime;

          System.out.println("Multi-Agent FullInfoX with k=" + k + ": " + (multiTime / 1000.0) + " sec");
          results.add("FullInfoX_Multi," + graphSize + "," + edges + "," + k + "," + (multiTime / 1000.0));
      }

      // Print results to console
      System.out.println("=== Table Results ===");
      System.out.println("algorithm,graph_size,edges,agents,time_seconds");
      for (String line : results) {
          System.out.println(line);
      }

      // Write results to file
      File file = new File("results.csv");
      boolean fileExists = file.exists() && file.length() > 0;

      try (PrintWriter out = new PrintWriter(new FileWriter(file, true))) {
          if (!fileExists) {
              out.println("algorithm,graph_size,edges,agents,time_seconds");
          }
          // Print each result line
          for (String line : results) {
              out.println(line);
          }
      }

      System.out.println("Results appended to results.csv");
  }
}
