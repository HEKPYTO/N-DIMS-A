package algo;

import java.util.*;

/**
 * An implementation of the A* pathfinding algorithm for N-dimensional space.
 * This class provides functionality to find the shortest path between two points
 * in an N-dimensional space while avoiding obstacles.
 *
 * <p>The implementation uses Manhattan distance as the heuristic function and
 * supports arbitrary dimensions. Memory usage scales exponentially with the
 * number of dimensions, so care should be taken with high-dimensional spaces.</p>
 *
 * <p>Example usage:</p>
 * <pre>
 * int[] start = {0, 0, 0};
 * int[] goal = {5, 5, 5};
 * int[] dimensions = {6, 6, 6};
 * Set<String> obstacles = new HashSet<>();
 * obstacles.add("[1, 1, 1]");
 *
 * List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);
 * </pre>
 *
 * @author [Your Name]
 * @version 1.0
 */
public class NDimensionalAstar {

    /**
     * Represents a node in N-dimensional space with path-finding metadata.
     * This class maintains the state necessary for A* pathfinding including
     * position coordinates and various cost calculations.
     */
    private static class Node {
        /** The N-dimensional coordinates of this node */
        private final int[] coordinates;

        /** The cost of the path from the start node to this node */
        private double gCost;

        /** The estimated cost from this node to the goal using the heuristic function */
        private double hCost;

        /** Reference to the previous node in the optimal path */
        private Node parent;

        /**
         * Constructs a new Node with the specified coordinates.
         *
         * @param coordinates The N-dimensional coordinates of the node
         */
        public Node(int[] coordinates) {
            this.coordinates = coordinates.clone();
            this.gCost = Double.MAX_VALUE;
            this.hCost = 0;
            this.parent = null;
        }

        /**
         * Calculates the total estimated cost for this node.
         * F-cost is the sum of the cost to reach this node (g-cost) and
         * the estimated cost to reach the goal from this node (h-cost).
         *
         * @return The total estimated cost (f-cost) for this node
         */
        public double getFCost() {
            return gCost + hCost;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Node node = (Node) o;
            return Arrays.equals(coordinates, node.coordinates);
        }

        @Override
        public int hashCode() {
            return Arrays.hashCode(coordinates);
        }
    }

    /**
     * Calculates the Manhattan distance between two points in N-dimensional space.
     * The Manhattan distance is the sum of the absolute differences of coordinates
     * in each dimension.
     *
     * @param start The starting coordinates
     * @param end The ending coordinates
     * @return The Manhattan distance between the two points
     */
    private static double manhattanDistance(int[] start, int[] end) {
        double distance = 0;
        for (int i = 0; i < start.length; i++) {
            distance += Math.abs(start[i] - end[i]);
        }
        return distance;
    }

    /**
     * Generates valid neighboring nodes in N-dimensional space.
     * A valid neighbor is one step away in any dimension and:
     * - Within the boundaries of the space
     * - Not occupied by an obstacle
     * - Reachable by moving along a single dimension
     *
     * @param current The current node
     * @param dimensions The size limits of each dimension
     * @param obstacles Set of coordinates where obstacles are present
     * @return List of valid neighboring nodes
     */
    private static List<Node> getNeighbors(Node current, int[] dimensions, Set<String> obstacles) {
        List<Node> neighbors = new ArrayList<>();
        int n = current.coordinates.length;

        for (int dim = 0; dim < n; dim++) {
            for (int delta : new int[]{-1, 1}) {
                int[] newCoords = current.coordinates.clone();
                newCoords[dim] += delta;

                boolean valid = true;
                for (int i = 0; i < n; i++) {
                    if (newCoords[i] < 0 || newCoords[i] >= dimensions[i]) {
                        valid = false;
                        break;
                    }
                }

                if (valid && !obstacles.contains(Arrays.toString(newCoords))) {
                    neighbors.add(new Node(newCoords));
                }
            }
        }
        return neighbors;
    }

    /**
     * Finds the shortest path from start to goal in N-dimensional space using the A* algorithm.
     * The implementation uses a priority queue for selecting the most promising nodes and
     * Manhattan distance as the heuristic function.
     *
     * <p>The method performs extensive validation of inputs and throws appropriate exceptions
     * if the inputs are invalid. It also includes a maximum iteration limit to prevent
     * infinite loops in pathological cases.</p>
     *
     * @param start Starting coordinates in N-dimensional space
     * @param goal Goal coordinates in N-dimensional space
     * @param dimensions Size of each dimension in the space
     * @param obstacles Set of coordinates where obstacles are present (as strings)
     * @return List of coordinates representing the path, or null if no path exists
     * @throws IllegalArgumentException if:
     *         - Any input array is null
     *         - Dimension counts don't match between start, goal, and dimensions arrays
     *         - Any coordinate is out of bounds
     *         - Start or goal position is occupied by an obstacle
     */
    public static List<int[]> findPath(int[] start, int[] goal, int[] dimensions, Set<String> obstacles) {
        // Input validation
        if (start == null || goal == null || dimensions == null) {
            throw new IllegalArgumentException("Input arrays cannot be null");
        }
        if (start.length != goal.length || start.length != dimensions.length) {
            throw new IllegalArgumentException("Dimensions mismatch: start=" + start.length +
                    ", goal=" + goal.length + ", dimensions=" + dimensions.length);
        }

        // Boundary validation
        for (int i = 0; i < dimensions.length; i++) {
            if (start[i] < 0 || start[i] >= dimensions[i]) {
                throw new IllegalArgumentException("Start coordinate out of bounds at dimension " + i);
            }
            if (goal[i] < 0 || goal[i] >= dimensions[i]) {
                throw new IllegalArgumentException("Goal coordinate out of bounds at dimension " + i);
            }
        }

        // Obstacle validation
        if (obstacles.contains(Arrays.toString(start))) {
            throw new IllegalArgumentException("Start position is an obstacle");
        }
        if (obstacles.contains(Arrays.toString(goal))) {
            throw new IllegalArgumentException("Goal position is an obstacle");
        }

        // Special case: start equals goal
        if (Arrays.equals(start, goal)) {
            List<int[]> path = new ArrayList<>();
            path.add(start.clone());
            return path;
        }

        // Initialize A* algorithm
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(Node::getFCost));
        Set<String> closedSet = new HashSet<>();

        Node startNode = new Node(start);
        startNode.gCost = 0;
        startNode.hCost = manhattanDistance(start, goal);

        openSet.add(startNode);
        int iterations = 0;
        int maxIterations = Math.max(10000, (int)Math.pow(2, start.length) * 10);

        // Main A* loop
        while (!openSet.isEmpty() && iterations < maxIterations) {
            iterations++;
            Node current = openSet.poll();

            if (Arrays.equals(current.coordinates, goal)) {
                return reconstructPath(current);
            }

            closedSet.add(Arrays.toString(current.coordinates));

            for (Node neighbor : getNeighbors(current, dimensions, obstacles)) {
                if (closedSet.contains(Arrays.toString(neighbor.coordinates))) {
                    continue;
                }

                double tentativeGCost = current.gCost + 1;

                boolean inOpenSet = openSet.contains(neighbor);
                if (!inOpenSet || tentativeGCost < neighbor.gCost) {
                    neighbor.parent = current;
                    neighbor.gCost = tentativeGCost;
                    neighbor.hCost = manhattanDistance(neighbor.coordinates, goal);

                    if (!inOpenSet) {
                        openSet.add(neighbor);
                    }
                }
            }
        }

        return null; // No path found
    }

    /**
     * Reconstructs the path from the goal node back to the start node.
     * This method follows the parent references from the goal node back
     * to the start node to build the complete path.
     *
     * @param goal The goal node from which to reconstruct the path
     * @return List of coordinate arrays representing the path from start to goal
     */
    private static List<int[]> reconstructPath(Node goal) {
        List<int[]> path = new ArrayList<>();
        Node current = goal;
        while (current != null) {
            path.add(0, current.coordinates.clone());
            current = current.parent;
        }
        return path;
    }
}