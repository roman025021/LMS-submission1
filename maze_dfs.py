"""Task 8 demo: DFS (Left>Straight>Right priority, visited-node memory) on the example maze,
then shortest path on the DISCOVERED graph and the turn list for replay.  python3 maze_dfs.py"""
import heapq

# node -> (x, y).  y grows to the NORTH.
POS = {'S':(0,0),'A':(0,1),'B':(0,2),'H':(-1,2),'D':(1,2),'C':(1,1),'E':(2,1)}
EDGES = [('S','A'),('A','B'),('A','C'),('B','H'),('B','D'),('C','D'),('C','E')]   # each 30 cm
GOAL = 'E'
DIRS = {'N':(0,1),'E':(1,0),'S':(0,-1),'W':(-1,0)}; ORDER = 'NESW'
adj = {n:{} for n in POS}                      # ground truth (robot can't see this)
for a,b in EDGES:
    for u,v in ((a,b),(b,a)):
        dx,dy = POS[v][0]-POS[u][0], POS[v][1]-POS[u][1]
        adj[u][[k for k,d in DIRS.items() if d==(dx,dy)][0]] = v

def rel(h, turn):                               # heading after turning L/S/R/B
    i = ORDER.index(h); return ORDER[(i + {'L':-1,'S':0,'R':1,'B':2}[turn]) % 4]

def fresh():
    global visited, trace, known
    visited, trace, known = set(), [], {n:set() for n in POS}
EXPLORE_ALL = False
def dfs(node, heading):
    visited.add(node); trace.append(node)
    if node == GOAL and not EXPLORE_ALL: return True   # stop at goal (mode 1) or keep mapping (mode 2)
    for t in 'LSR':                             # priority: Left, Straight, Right
        h = rel(heading, t)
        nxt = adj[node].get(h)
        if nxt is None: continue
        known[node].add(nxt); known[nxt].add(node)       # discovered edge (also records LOOP edges)
        if nxt in visited: continue                      # already explored -> skip (loop detected)
        if dfs(nxt, h): return True
        trace.append(node)                               # backtrack: physically drive back to node
    return False

def report():
    # --- optimal route on the discovered graph (Dijkstra, edge cost = 30 cm; add turn costs here if wanted)
    def dijkstra(src, dst):
        pq=[(0,src,[src])]; seen=set()
        while pq:
            c,n,p = heapq.heappop(pq)
            if n==dst: return c,p
            if n in seen: continue
            seen.add(n)
            for m in known[n]: heapq.heappush(pq,(c+30,m,p+[m]))
    cost, path = dijkstra('S', GOAL)
    print(f"Shortest known path: {' -> '.join(path)}  ({cost} cm)")

    # --- turn list for replay: only the decision at each junction on the path matters
    def heading_of(a,b): return [k for k,v in adj[a].items() if v==b][0]
    h='N'; turns=[]
    for i in range(1,len(path)-1):
        nh = heading_of(path[i], path[i+1]); ih = ORDER.index(h); jh = ORDER.index(nh)
        turns.append((path[i], {0:'S',1:'R',3:'L',2:'B'}[(jh-ih)%4])); h = nh
    print("Replay turns at each node:", turns)

for EXPLORE_ALL in (False, True):
    fresh(); dfs('S','N')
    print(f"\n=== mode: {'map EVERYTHING (continue after goal)' if EXPLORE_ALL else 'stop at goal'} ===")
    print("DFS walk (incl. backtracking):", ' -> '.join(trace))
    print("Discovered edges:", sorted({tuple(sorted((a,b))) for a in known for b in known[a]}))
    report()

