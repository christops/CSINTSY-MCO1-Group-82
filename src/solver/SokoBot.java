package solver;

import java.util.*;

public class SokoBot {

  /* -0-0-0-0-0-0-0-0-0-0- FUNCTIONS -0-0-0-0-0-0-0-0-0-0- */

  /**
   * solveSokobanPuzzle
   * parses the map into usable structures and runs A* to find a solution.
   * @param width     number of columns in the level
   * @param height    number of rows in the level
   * @param mapData   2D char array of static elements
   * @param itemsData 2D char array of moving elements
   * @return          solution string of moves, or empty string if unsolvable
   */
  public String solveSokobanPuzzle(int width, int height, char[][] mapData, char[][] itemsData) {
    // collect map info
    boolean[][] walls = new boolean[height][width];
    boolean[][] targets = new boolean[height][width];
    boolean[][] deadCells = new boolean[height][width];

    List<int[]> targetList = new ArrayList<>();

    for (int r = 0; r < height; r++)
      for (int c = 0; c < width; c++) {
        if (mapData[r][c] == '#')
          walls[r][c] = true;
        if (mapData[r][c] == '.') {
          targets[r][c] = true;
          targetList.add(new int[]{r,c});
        }
      }

    computeDeadCells(deadCells, walls, targets, height, width);

    // build initial state
    int playerR = -1, playerC = -1;
    List<long[]> initBoxes = new ArrayList<>();

    for (int r = 0; r < height; r++)
      for (int c = 0; c < width; c++) {
        if (itemsData[r][c] == '@') {
          playerR = r;
          playerC = c;
        }
        if (itemsData[r][c] == '$')
          initBoxes.add(new long[]{r,c});
      }

    long[] boxArr = new long[initBoxes.size()];

    for (int i = 0; i < initBoxes.size(); i++)
      boxArr[i] = pack(initBoxes.get(i)[0], initBoxes.get(i)[1]);

    Arrays.sort(boxArr);
    State initial = new State(playerR, playerC, boxArr, "");
    String result = astar(initial, walls, targets, targetList, deadCells, height, width);

    if (result == null) {
      return "";
    } else {
      return result;
    }
  }

  /**
   * astar
   * runs A* search over the Sokoban state space to find an optimal sequence of moves.
   * @param initial    the starting state
   * @param walls      2D boolean array marking wall cells
   * @param targets    2D boolean array marking target cells
   * @param targetList flat list of target coordinates for heuristic computation
   * @param deadCells  2D boolean array of statically unreachable cells (see computeDeadCells)
   * @param height     number of rows in the level
   * @param width      number of columns in the level
   * @return           solution move string, or null if no solution exists
   */
  private String astar(State initial, boolean[][] walls, boolean[][] targets,
                       List<int[]> targetList, boolean[][] deadCells, int height, int width) {

    PriorityQueue<State> open = new PriorityQueue<>(Comparator.comparingInt(s -> s.f));
    Map<Long, Integer> visited = new HashMap<>();

    initial.g = 0;
    initial.h = heuristic(initial.boxes, targetList);
    initial.f = initial.h;
    open.add(initial);

    int[] dr = {-1, 1, 0, 0};
    int[] dc = {0, 0, -1, 1};
    char[] dirChar = {'u','d','l','r'};

    while (!open.isEmpty()) {
      State cur = open.poll();

      // goal check
      if (isGoal(cur.boxes, targets))
        return cur.path;

      long hash = stateHash(cur);
      Integer best = visited.get(hash);

      if (best != null && best <= cur.g)
        continue;

      visited.put(hash, cur.g);

      // expand
      for (int d = 0; d < 4; d++) {
        int nr = cur.playerR + dr[d];
        int nc = cur.playerC + dc[d];

        if (nr < 0 || nr >= height || nc < 0 || nc >= width)
          continue;
        if (walls[nr][nc])
          continue;

        int boxIdx = findBox(cur.boxes, nr, nc);
        long[] newBoxes = cur.boxes;

        if (boxIdx >= 0) {
          // push a box
          int br = nr + dr[d];
          int bc = nc + dc[d];

          if (br < 0 || br >= height || bc < 0 || bc >= width)
            continue;
          if (walls[br][bc])
            continue;
          if (findBox(cur.boxes, br, bc) >= 0)
            continue;
          if (deadCells[br][bc] && !targets[br][bc])
            continue;

          newBoxes = cur.boxes.clone();
          newBoxes[boxIdx] = pack(br, bc);
          Arrays.sort(newBoxes);

          if (isFrozenDeadlock(newBoxes, walls, targets, br, bc, height, width))
            continue;
        }

        int newG = cur.g + 1;
        State next = new State(nr, nc, newBoxes, cur.path + dirChar[d]);
        next.g = newG;
        next.h = heuristic(newBoxes, targetList);
        next.f = newG + next.h;

        long nextHash = stateHash(next);
        Integer nextBest = visited.get(nextHash);

        if (nextBest != null && nextBest <= newG)
          continue;

        open.add(next);
      }
    }
    return null; // if the program exits the while-loop, there is no solution
  }

  /**
   * heuristic
   * estimates remaining cost as the sum of each box's Manhattan distance to its nearest target.
   * @param boxes   sorted array of packed box positions
   * @param targets list of target coordinates
   * @return        estimated number of moves still needed
   */
  private int heuristic(long[] boxes, List<int[]> targets) {
    int total = 0;

    for (long b : boxes) {
      int br = row(b), bc = col(b);
      int minDist = Integer.MAX_VALUE;

      for (int[] t : targets) {
        int d = Math.abs(br - t[0]) + Math.abs(bc - t[1]);

        if (d < minDist)
          minDist = d;
      }
      total += minDist;
    }
    return total;
  }

  /**
   * computeDeadCells
   * pre-computes which cells are statically dead (a box there can never reach any target).
   * @param dead    output array; dead[r][c] is set to true if the cell is a dead cell
   * @param walls   2D boolean array marking wall cells
   * @param targets 2D boolean array marking target cells
   * @param height  number of rows in the level
   * @param width   number of columns in the level
   * @return        void (results written into dead[][])
   */
  private void computeDeadCells(boolean[][] dead, boolean[][] walls,
                                boolean[][] targets, int height, int width) {
    // if a box placed in a cell can be pushed to a target, it is "live"
    // compute live cells using reverse BFS (pull boxes from targets)
    boolean[][] live = new boolean[height][width];
    Queue<long[]> queue = new LinkedList<>();

    // seed (all target cells are live)
    for (int r = 0; r < height; r++)
      for (int c = 0; c < width; c++)
        if (targets[r][c]) {
          live[r][c] = true;
          queue.add(new long[]{r,c});
        }

    int[] dr = {-1,1,0,0};
    int[] dc = {0,0,-1,1};

    while (!queue.isEmpty()) {
      long[] cur = queue.poll();
      int r = (int)cur[0], c = (int)cur[1];

      for (int d = 0; d < 4; d++) {
        int br = r - dr[d]; int bc = c - dc[d]; // where box was
        int pr = r + dr[d]; int pc = c + dc[d]; // where player was

        if (br < 0 || br >= height || bc < 0 || bc >= width)
          continue;
        if (pr < 0 || pr >= height || pc < 0 || pc >= width)
          continue;
        if (walls[br][bc] || walls[pr][pc])
          continue;
        if (!live[br][bc]) {
          live[br][bc] = true;
          queue.add(new long[]{br, bc});
        }
      }
    }

    for (int r = 0; r < height; r++)
      for (int c = 0; c < width; c++)
        if (!walls[r][c])
          dead[r][c] = !live[r][c];
  }

  /**
   * isFrozenDeadlock
   * checks whether the newly pushed box causes a 2x2 freeze deadlock.
   * A 2x2 freeze deadlock is a 2x2 block where every cell is a wall or box, and at least one box is off a target.
   * @param boxes   current sorted array of packed box positions
   * @param walls   2D boolean array marking wall cells
   * @param targets 2D boolean array marking target cells
   * @param br      row of the newly pushed box
   * @param bc      column of the newly pushed box
   * @param height  number of rows in the level
   * @param width   number of columns in the level
   * @return        true if a freeze deadlock is detected, false otherwise
   */
  private boolean isFrozenDeadlock(long[] boxes, boolean[][] walls, boolean[][] targets,
                                   int br, int bc, int height, int width) {
    // check all 2x2 squares containing (br,bc)
    int[] offR = {0, 0, -1, -1};
    int[] offC = {0, -1, 0, -1};

    for (int k = 0; k < 4; k++) {
      int r0 = br + offR[k];
      int c0 = bc + offC[k];

      if (r0 < 0 || r0+1 >= height || c0 < 0 || c0+1 >= width)
        continue;

      // check 2x2 block (r0,c0),(r0,c0+1),(r0+1,c0),(r0+1,c0+1)
      boolean allBlocked = true;
      boolean hasBox = false;
      boolean allOnTarget = true;

      for (int dr2 = 0; dr2 <= 1; dr2++) {
        for (int dc2 = 0; dc2 <= 1; dc2++) {
          int rr = r0+dr2, cc = c0+dc2;
          boolean isWall = walls[rr][cc];
          boolean isBox  = findBox(boxes, rr, cc) >= 0;

          if (!isWall && !isBox) {
            allBlocked = false;
            break;
          }
          if (isBox) {
            hasBox = true;
            if (!targets[rr][cc])
              allOnTarget = false;
          }
        }
        if (!allBlocked)
          break;
      }
      if (allBlocked && hasBox && !allOnTarget)
        return true;
    }
    return false;
  }

  /* -0-0-0-0-0-0-0-0-0-0- HELPERS -0-0-0-0-0-0-0-0-0-0- */

  /**
   * isGoal
   * checks whether all boxes are on target cells.
   * @param boxes   sorted array of packed box positions
   * @param targets 2D boolean array marking target cells
   * @return        true if puzzle is solved, false otherwise
   */
  private boolean isGoal(long[] boxes, boolean[][] targets) {
    for (long b : boxes)
      if (!targets[row(b)][col(b)])
        return false;
    return true;
  }

  /**
   * findBox
   * searches the boxes array for a box at the given coordinates.
   * @param boxes sorted array of packed box positions
   * @param r     row to search
   * @param c     column to search
   * @return      index of the box in the array, or -1 if not found
   */
  private int findBox(long[] boxes, int r, int c) {
    long key = pack(r, c);
    for (int i = 0; i < boxes.length; i++)
      if (boxes[i] == key)
        return i;
    return -1;
  }

  /**
   * pack
   * encodes a (row, col) pair into a single long value for compact storage and hashing.
   * @param r row coordinate
   * @param c column coordinate
   * @return  encoded long value
   */
  private long pack(long r, long c) {
    return r * 1000 + c;
  }

  /**
   * row
   * extracts the row from a packed coordinate.
   * @param p packed coordinate
   * @return  row value
   */
  private int row(long p) {
    return (int)(p / 1000);
  }

  /**
   * col
   * extracts the column from a packed coordinate.
   * @param p packed coordinate
   * @return  column value
   */
  private int col(long p) {
    return (int)(p % 1000);
  }

  /**
   * stateHash
   * produces a single long hash representing the full game state.
   * @param s the state to hash
   * @return  hash value of the state
   */
  private long stateHash(State s) {
    long h = s.playerR * 1000L + s.playerC;
    for (long b : s.boxes)
      h = h * 1_000_003L + b;
    return h;
  }

  /* -0-0-0-0-0-0-0-0-0-0- STATE CLASS -0-0-0-0-0-0-0-0-0-0- */

  /**
   * State
   * represents a single snapshot of the game at a point in time.
   * Stores; [1] the player's position, [2] all box positions, [3] the move path taken to reach this state,
   * and [4] the A* cost values used to prioritize exploration.
   *
   * @field playerR - current row  of the player [1]
   * @field playerC - current column of the player [1]
   * @field boxes   - sorted array of packed box positions (see pack()) [2]
   * @field path    - sequence of moves taken from the initial state to reach this one [3]
   * @field g       - actual cost (number of moves taken so far) [4]
   * @field h       - estimated cost to goal (heuristic) [4]
   * @field f       - total estimated cost (f = g + h) [4]
   */
  private static class State {
    int playerR, playerC;
    long[] boxes;
    String path;
    int g, h, f;

    State(int pr, int pc, long[] boxes, String path) {
      this.playerR = pr; this.playerC = pc;
      this.boxes = boxes; this.path = path;
    }
  }
}
