# N-Dimensional A* Pathfinding Algorithm

An efficient implementation of the A* pathfinding algorithm that works in N-dimensional space. The algorithm finds the shortest path between two points while avoiding obstacles, with support for arbitrary dimensions.

## Overview

This implementation provides:
- N-dimensional pathfinding with Manhattan distance heuristic
- Efficient obstacle avoidance
- Comprehensive input validation
- Support for asymmetric dimension sizes
- Path optimization for high-dimensional spaces

## Usage Example

```java
// Define a 3D space
int[] start = {0, 0, 0};
int[] goal = {5, 5, 5};
int[] dimensions = {6, 6, 6};

// Add some obstacles
Set<String> obstacles = new HashSet<>();
obstacles.add("[1, 1, 1]");
obstacles.add("[2, 2, 2]");

// Find the path
List<int[]> path = NDimensionalAstar.findPath(start, goal, dimensions, obstacles);

// Print the path
if (path != null) {
    for (int[] point : path) {
        System.out.println(Arrays.toString(point));
    }
} else {
    System.out.println("No path found!");
}
```

## Project Structure

```
└── src/
    ├── algo/
    │   └── NDimensionalAstar.java    // Main algorithm implementation
    └── test/
        └── testNdimsAstar.java       // JUnit test suite
```

## Tests

The test suite (`testNdimsAstar.java`) includes comprehensive tests for:

1. **Dimensional Testing**
   - 4D, 5D, and 6D space pathfinding
   - Variable dimension sizes (up to 15D)
   - Asymmetric dimension handling

2. **Edge Cases**
   - Start equals goal
   - No path exists
   - Out of bounds coordinates
   - Invalid dimensions

3. **Performance Tests**
   - Path continuity verification
   - Large dimension handling (up to 15D)
   - Time limit constraints

4. **Obstacle Handling**
   - Multiple obstacle configurations
   - Path validation around obstacles
   - Complex obstacle patterns

To run tests:
```bash
javac -cp junit-jupiter-api-5.x.x.jar test/testNdimsAstar.java
java -cp junit-jupiter-engine-5.x.x.jar org.junit.platform.console.ConsoleLauncher --scan-classpath
```

## Performance Considerations

- Memory usage scales exponentially with the number of dimensions
- Practical limit of around 15 dimensions for reasonable performance
- Efficient for smaller dimensional spaces (2D-10D)
- Uses Manhattan distance heuristic for path optimization

## Limitations

- Memory intensive for high dimensions (>10D)
- Performance degrades exponentially with dimension count
- Maximum recommended dimension size: 1000
- Maximum recommended dimension count: 15
