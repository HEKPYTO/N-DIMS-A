package test;

import algo.NDimensionalAstar;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.*;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

public class testNdimsAstar {
    @Test
    @DisplayName("Test path finding in 4D space")
    void test4DPathFinding() {
        int[] start = {0, 0, 0, 0};
        int[] goal = {1, 1, 1, 1};
        int[] dimensions = {2, 2, 2, 2};
        Set<String> obstacles = new HashSet<>();

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

        assertNotNull(path, "Path should exist in 4D space");
        assertTrue(!path.isEmpty(), "Path should not be empty");
        assertArrayEquals(start, path.get(0), "Path should start at start position");
        assertArrayEquals(goal, path.get(path.size() - 1), "Path should end at goal position");
    }

    @Test
    @DisplayName("Test path finding in 5D space")
    void test5DPathFinding() {
        int[] start = {0, 0, 0, 0, 0};
        int[] goal = {1, 1, 1, 1, 1};
        int[] dimensions = {2, 2, 2, 2, 2};
        Set<String> obstacles = new HashSet<>();

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

        assertNotNull(path, "Path should exist in 5D space");
        assertTrue(path.size() > 0, "Path should not be empty");
        assertArrayEquals(start, path.get(0), "Path should start at start position");
        assertArrayEquals(goal, path.get(path.size() - 1), "Path should end at goal position");
    }

    @Test
    @DisplayName("Test path finding in 6D space with obstacles")
    void test6DPathFindingWithObstacles() {
        int[] start = {0, 0, 0, 0, 0, 0};
        int[] goal = {1, 1, 1, 1, 1, 1};
        int[] dimensions = {2, 2, 2, 2, 2, 2};
        Set<String> obstacles = new HashSet<>();
        obstacles.add(Arrays.toString(new int[]{0, 1, 0, 1, 0, 1}));
        obstacles.add(Arrays.toString(new int[]{1, 0, 1, 0, 1, 0}));

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

        assertNotNull(path, "Path should exist in 6D space with obstacles");
        assertTrue(path.size() > 0, "Path should not be empty");
        assertArrayEquals(start, path.get(0), "Path should start at start position");
        assertArrayEquals(goal, path.get(path.size() - 1), "Path should end at goal position");

        // Verify no obstacles in path
        for (int[] point : path) {
            assertFalse(obstacles.contains(Arrays.toString(point)),
                    "Path should not contain obstacles: " + Arrays.toString(point));
        }
    }

    // Test cases for path finding with various dimensions
    @ParameterizedTest(name = "Test path finding in {0}D space")
    @MethodSource("provideDimensions")
    @DisplayName("Test path finding with various dimensions")
    void testVariousDimensions(int dimension) {
        int[] start = new int[dimension];
        int[] goal = new int[dimension];
        int[] dimensions = new int[dimension];

        Arrays.fill(dimensions, 2);
        Arrays.fill(goal, 1);

        Set<String> obstacles = new HashSet<>();

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

        assertNotNull(path, "Path should exist in " + dimension + "D space");
        assertTrue(path.size() > 0, "Path should not be empty");
        assertArrayEquals(start, path.get(0), "Path should start at start position");
        assertArrayEquals(goal, path.get(path.size() - 1), "Path should end at goal position");
    }

    private static Stream<Arguments> provideDimensions() {
        return Stream.of(
                Arguments.of(4),
                Arguments.of(5),
                Arguments.of(6),
                Arguments.of(7),
                Arguments.of(8),
                Arguments.of(10)
        );
    }

    // Test cases for edge cases
    @Test
    @DisplayName("Test no path exists in high dimensions")
    void testNoPathInHighDimensions() {
        int[] start = new int[8];
        int[] goal = new int[8];
        int[] dimensions = new int[8];
        Arrays.fill(dimensions, 2);
        Arrays.fill(goal, 1);

        Set<String> obstacles = new HashSet<>();

        // Create a wall of obstacles blocking all possible paths
        for (int i = 0; i < 8; i++) {
            int[] obstacle = new int[8];
            obstacle[i] = 1;
            obstacles.add(Arrays.toString(obstacle));
        }

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);
        assertNull(path, "Path should not exist when blocked by obstacles in 8D space");
    }

    @Test
    @DisplayName("Test path finding with asymmetric dimensions")
    void testAsymmetricDimensions() {
        int[] start = {0, 0, 0, 0};
        int[] goal = {2, 3, 1, 4};
        int[] dimensions = {3, 4, 2, 5};
        Set<String> obstacles = new HashSet<>();

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

        assertNotNull(path, "Path should exist in asymmetric 4D space");
        assertTrue(path.size() > 0, "Path should not be empty");
        assertArrayEquals(start, path.get(0), "Path should start at start position");
        assertArrayEquals(goal, path.get(path.size() - 1), "Path should end at goal position");
    }

    @Test
    @DisplayName("Test edge case: Start equals goal in high dimensions")
    void testStartEqualsGoalHighDimensions() {
        int dimension = 10;
        int[] start = new int[dimension];
        int[] goal = new int[dimension];
        int[] dimensions = new int[dimension];
        Arrays.fill(dimensions, 2);

        Set<String> obstacles = new HashSet<>();

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

        assertNotNull(path, "Path should exist when start equals goal");
        assertEquals(1, path.size(), "Path should contain only one point when start equals goal");
        assertArrayEquals(start, path.get(0), "Path should contain only start/goal point");
    }

    // Test cases for invalid inputs
    @Test
    @DisplayName("Test invalid dimensions")
    void testInvalidDimensions() {
        assertThrows(IllegalArgumentException.class, () -> {
            int[] start = {0, 0, 0};
            int[] goal = {0, 0, 0, 0};
            int[] dimensions = {1, 1, 1, 1};
            NDimensionalAstar.findPath(start, goal, dimensions, new HashSet<>());
        }, "Should throw exception for mismatched dimensions");
    }

    @Test
    @DisplayName("Test out of bounds in high dimensions")
    void testOutOfBoundsHighDimensions() {
        int dimension = 8;
        int[] start = new int[dimension];
        int[] goal = new int[dimension];
        int[] dimensions = new int[dimension];
        Arrays.fill(dimensions, 2);

        // Set one coordinate out of bounds
        goal[dimension - 1] = 2;

        assertThrows(IllegalArgumentException.class, () -> {
            NDimensionalAstar.findPath(start, goal, dimensions, new HashSet<>());
        }, "Should throw exception for out of bounds coordinates");
    }

    // Test cases for performance
    @Test
    @DisplayName("Test path continuity in high dimensions")
    void testPathContinuityHighDimensions() {
        int[] start = new int[6];
        int[] goal = new int[6];
        int[] dimensions = new int[6];
        Arrays.fill(dimensions, 3);
        Arrays.fill(goal, 2);

        Set<String> obstacles = new HashSet<>();

        List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

        assertNotNull(path, "Path should exist");

        // Verify path continuity (each point should differ by at most one coordinate)
        for (int i = 1; i < path.size(); i++) {
            int[] current = path.get(i);
            int[] previous = path.get(i - 1);
            int differences = 0;

            for (int j = 0; j < current.length; j++) {
                if (current[j] != previous[j]) {
                    differences++;
                }
            }

            assertEquals(1, differences,
                    "Each step should only change one coordinate\n" +
                            "Previous: " + Arrays.toString(previous) + "\n" +
                            "Current: " + Arrays.toString(current));
        }
    }

    @Test
    @DisplayName("Test performance with large dimensions")
    void testPerformanceLargeDimensions() {
        int dimension = 15;
        int[] start = new int[dimension];
        int[] goal = new int[dimension];
        int[] dimensions = new int[dimension];
        Arrays.fill(dimensions, 2);
        Arrays.fill(goal, 1);

        Set<String> obstacles = new HashSet<>();

        assertTimeoutPreemptively(java.time.Duration.ofSeconds(5), () -> {
            List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);
            assertNotNull(path, "Should find path in reasonable time");
        }, "Path finding should complete within reasonable time");
    }
}
