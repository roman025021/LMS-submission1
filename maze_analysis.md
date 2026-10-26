# Maze-solving analysis (Task 8)

## 1. DFS vs wall-preference (left-hand rule)
| | Left-hand rule | DFS with visited-junction memory |
|---|---|---|
| Memory | none | map (nodes, edges) - a few hundred bytes |
| Complete? | only guaranteed when the maze has no loops (tree / simply connected). In a graph with loops it follows one face boundary: it can circle forever and never reach a goal/spur lying on another face | Yes: visits every reachable junction exactly once, handles loops |
| Output | one path, includes dead-end detours | a map -> can compute the true shortest path |
| Needs | line/junction detection only | plus junction identity (odometry/encoders, or a grid assumption) |
| Complexity | trivial | O(V+E) |

**Choice for an unknown maze: DFS with visited memory** (priority Left > Straight > Right, so it degrades to the left-hand rule when no loop is encountered). Left-hand rule is my fallback if I can't localise junctions. Cost: needs reliable junction identification.

## 2. Example maze (30 cm per edge, goal E, robot starts at S facing north)
```
y=2    H ──── B ──── D
              │      │
y=1           A ──── C ──── E (goal)
              │
y=0           S (start)
      x=-1   x=0    x=1    x=2
```
* **Nodes** = junctions, corners, dead ends, start, goal (S, A, B, H, D, C, E). **Edges** = line segments between them, weight = length (30 cm).
* Edges: S-A, A-B, A-C, B-H, B-D, C-D, C-E. Cycle: A-B-D-C-A. H is a dead end.

## 3. DFS traversal (run `src/maze/maze_dfs.py`)
Priority at each node: Left, Straight, Right, relative to heading; skip visited nodes.
1. S -> A (heading N). At A: left (W) none; straight (N) -> B.
2. At B (N): left (W) -> H. H is a dead end: U-turn, backtrack to B.
3. At B (now E): straight (E) -> D. At D: left/straight none, right (S) -> C.
4. At C (S): left (E) -> E = **goal**. Walk: `S A B H B D C E` (150 cm path + 60 cm of detour).
5. Keep exploring (mapping run): from C the right branch (W) leads to A, already visited -> **loop closure**, edge A-C recorded; backtrack to S.
6. Discovered graph now includes A-C. **Dijkstra** on it: S-A-C-E = 90 cm vs 150 cm explored.
7. Replay turn list: at A turn Right, at C go Straight.

Note the stop-at-goal run only learns 150 cm path (A-C never seen); to get the true shortest path the exploration must continue (as in step 5), or accept the 150 cm route.

```
DFS(node, heading):
  mark node visited; if node == GOAL: record
  for turn in [LEFT, STRAIGHT, RIGHT]:
      h = heading + turn
      if branch exists at h:
          record edge(node, neighbour(h))
          if neighbour not visited:
              drive to neighbour; DFS(neighbour, h); drive back, face original heading
  # dead end: branch list empty -> return, physically U-turn
```

## 4. Exploration vs replay
**Exploration** is traversal under uncertainty: the robot must sense every junction, decide, remember, and physically backtrack, so it needs a real algorithm and a conservative speed. **Replay** is deterministic: the map is known, so the best route is precomputed (Dijkstra, edge cost = length + turn penalty) and reduced to a list of turn decisions (`[R, S]`). The robot only has to **count junctions and apply the next stored turn** - no search, no memory of visited nodes, no dead-end logic - so it can run much faster with the same line-following PID. The only extra hazard is miscounting a junction, so keep a checksum (expected junction type at each step).
