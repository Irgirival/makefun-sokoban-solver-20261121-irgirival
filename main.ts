// Sokoban solver — push-optimal per-room A*.
//
// One program answers a whole set of Sokoban levels (i001..i005). Each level is a
// hallway with 1-4 rooms off it; a box pushed out of a room's door can never be
// pushed back, so the rooms are independent problems joined only by the player's
// walk, and the level's push-optimal count is the sum of its rooms' optima.
// The solver therefore segments each level into rooms, solves every room
// push-optimally with A* (min-matching heuristic, dead-square + 2x2 deadlock
// pruning, player-region canonicalisation), and stitches the rooms together
// with shortest walks along the hallway.
//
// Node 22, standard library only. Fully deterministic: no clock, no randomness,
// no environment-dependent behaviour. The output depends only on the input files.
//
// Input:  /input/<id>/level.txt  (XSB: # wall, space floor, . goal, $ box,
//         * box on a goal, @ player; one row a line)
// Output: /output/<id>/solution.txt (the player's moves in LURD: u, d, l, r,
//         one letter a step; the replay decides which steps push a box).
//
// (Directories may be overridden with MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR,
// the harness convention; the answers never depend on them.)

import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { join } from 'node:path';

const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR || '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR || '/output';
const IDS = ['i001', 'i002', 'i003', 'i004', 'i005'];

// Work budgets (counts of expanded states, never the clock, so two runs agree).
const ASTAR_BUDGET = 250000; // per room, A*
const GREEDY_BUDGET = 250000; // per room, greedy fallback
const GLOBAL_BUDGET = 3000000; // whole set
let totalExpansions = 0;

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------

/** Neighbour of cell c in direction d, or -1 when it leaves the board. */
function nb(c, d, w, h) {
  const x = c % w;
  const y = (c / w) | 0;
  const nx = x + d.dx;
  const ny = y + d.dy;
  if (nx < 0 || nx >= w || ny < 0 || ny >= h) return -1;
  return ny * w + nx;
}

/** A binary min-heap of { f, c, key, g }; ties break by insertion order. */
class Heap {
  constructor() {
    this.a = [];
  }
  size() {
    return this.a.length;
  }
  push(item) {
    const a = this.a;
    a.push(item);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].f < item.f || (a[p].f === item.f && a[p].c <= item.c)) break;
      a[i] = a[p];
      i = p;
    }
    a[i] = item;
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      let i = 0;
      for (;;) {
        let l = 2 * i + 1;
        let r = l + 1;
        let m = i;
        if (l < a.length && (a[l].f < last.f || (a[l].f === last.f && a[l].c < last.c))) m = l;
        if (r < a.length && (a[r].f < a[m].f || (a[r].f === a[m].f && a[r].c < a[m].c))) m = r;
        if (m === i) break;
        a[i] = a[m];
        i = m;
      }
      a[i] = last;
    }
    return top;
  }
}

/** Flood fill from `from` over `cells` (a Set of passable indices), avoiding
 *  the cells in `blocked` (boxes). Returns the reached cells, sorted. */
function flood(cells, blocked, from, w, h) {
  const seen = new Set();
  if (!cells.has(from) || (blocked && blocked.has(from))) return [];
  const stack = [from];
  seen.add(from);
  while (stack.length > 0) {
    const c = stack.pop();
    for (const d of DIRS) {
      const n = nb(c, d, w, h);
      if (n < 0 || !cells.has(n) || seen.has(n)) continue;
      if (blocked && blocked.has(n)) continue;
      seen.add(n);
      stack.push(n);
    }
  }
  return [...seen].sort((a, b) => a - b);
}

/** Shortest walk (LURD letters) from `from` to `to` over `cells`, avoiding
 *  `blocked` (boxes). Null when unreachable. Deterministic. */
function walkPath(cells, blocked, from, to, w, h) {
  if (from === to) return '';
  const prev = new Map();
  prev.set(from, -1);
  const queue = [from];
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi];
    for (let di = 0; di < 4; di++) {
      const n = nb(c, DIRS[di], w, h);
      if (n < 0 || !cells.has(n) || prev.has(n)) continue;
      if (blocked && blocked.has(n)) continue;
      prev.set(n, { at: c, dir: di });
      if (n === to) {
        let out = '';
        let cur = to;
        while (prev.get(cur) !== -1) {
          const step = prev.get(cur);
          out = DIRS[step.dir].ch + out;
          cur = step.at;
        }
        return out;
      }
      queue.push(n);
    }
  }
  return null;
}

/** BFS distances from `from` over `cells` (ignoring boxes). -1 = unreachable. */
function bfsDist(cells, from, w, h) {
  const size = w * h;
  const dist = new Int32Array(size).fill(-1);
  dist[from] = 0;
  const queue = [from];
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi];
    for (const d of DIRS) {
      const n = nb(c, d, w, h);
      if (n < 0 || !cells.has(n) || dist[n] >= 0) continue;
      dist[n] = dist[c] + 1;
      queue.push(n);
    }
  }
  return dist;
}

// ---------------------------------------------------------------------------
// Parsing and room segmentation
// ---------------------------------------------------------------------------

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < rows[y].length; x++) {
      const ch = rows[y][x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

/** Split a level into its rooms. The hallway is the player's row; each room
 *  hangs off it through a single door square. Returns rooms in a fixed order:
 *  above rooms left to right, then below rooms left to right. */
function segmentRooms(L) {
  const { w, h, wall, goal, boxes, player } = L;
  const hy = (player / w) | 0;
  const boxSet = new Set(boxes);
  const rooms = [];
  const claimed = new Set();
  function floodSide(sx, sy, side) {
    const start = sy * w + sx;
    const cells = new Set();
    const stack = [start];
    cells.add(start);
    while (stack.length > 0) {
      const c = stack.pop();
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        if (side < 0 && ny >= hy) continue;
        if (side > 0 && ny <= hy) continue;
        const n = ny * w + nx;
        if (wall[n] || cells.has(n)) continue;
        cells.add(n);
        stack.push(n);
      }
    }
    return cells;
  }
  for (let x = 1; x < w - 1; x++) {
    for (const side of [-1, 1]) {
      const dy = side;
      const doorY = hy + dy;
      if (doorY < 0 || doorY >= h) continue;
      const door = doorY * w + x;
      if (wall[door] || claimed.has(door)) continue;
      const cells = floodSide(x, doorY, side);
      for (const c of cells) claimed.add(c);
      const roomBoxes = boxes.filter((b) => cells.has(b));
      const roomGoals = [];
      for (const c of cells) if (goal[c]) roomGoals.push(c);
      roomGoals.sort((a, b) => a - b);
      rooms.push({
        cells,
        door,
        doorOutside: hy * w + x,
        boxes: roomBoxes,
        goals: roomGoals,
      });
    }
  }
  return rooms;
}

// ---------------------------------------------------------------------------
// Per-room push-optimal search
// ---------------------------------------------------------------------------

/** Squares a box can ever be pushed from to reach a goal (reverse BFS from
 *  the goals, ignoring other boxes). A box off a live square is dead. */
function computeLive(cells, goals, w, h) {
  const live = new Uint8Array(w * h);
  for (const g of goals) live[g] = 1;
  const cellArr = [...cells];
  let changed = true;
  while (changed) {
    changed = false;
    for (const c of cellArr) {
      if (live[c]) continue;
      for (const d of DIRS) {
        const fwd = nb(c, d, w, h);
        const back = nb(c, { dx: -d.dx, dy: -d.dy }, w, h);
        if (fwd >= 0 && live[fwd] && back >= 0 && cells.has(back)) {
          live[c] = 1;
          changed = true;
          break;
        }
      }
    }
  }
  return live;
}

/** True when a 2x2 block of walls/boxes with no goal square freezes a box. */
function hasDeadBlock(boxes, boxSet, cells, goalSet, w, h) {
  for (const b of boxes) {
    const bx = b % w;
    const by = (b / w) | 0;
    for (let oy = -1; oy <= 0; oy++) {
      for (let ox = -1; ox <= 0; ox++) {
        let filled = 0;
        let hasGoal = false;
        for (let dy = 0; dy < 2; dy++) {
          for (let dx = 0; dx < 2; dx++) {
            const nx = bx + ox + dx;
            const ny = by + oy + dy;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
              filled++;
              continue;
            }
            const c = ny * w + nx;
            if (goalSet.has(c)) {
              hasGoal = true;
              continue;
            }
            if (!cells.has(c) || boxSet.has(c)) filled++;
          }
        }
        if (filled === 4 && !hasGoal) return true;
      }
    }
  }
  return false;
}

/** Minimum total box-to-goal distance over all assignments (brute force over
 *  the at most 4! bijections). Infinity when no box can reach a goal. This is
 *  an admissible, consistent heuristic: one push moves one box one square. */
function matchingHeuristic(boxes, dists, goals) {
  const n = boxes.length;
  const used = new Array(n).fill(false);
  let best = Infinity;
  (function rec(i, sum) {
    if (sum >= best) return;
    if (i === n) {
      best = sum;
      return;
    }
    for (let j = 0; j < n; j++) {
      if (used[j]) continue;
      const c = dists[j][boxes[i]];
      if (c < 0) continue;
      used[j] = true;
      rec(i + 1, sum + c);
      used[j] = false;
    }
  })(0, 0);
  return best;
}

function stateKey(boxes, regionRep) {
  return boxes.join(',') + '|' + regionRep;
}

/**
 * Push-optimal search for one room, from the player standing on the square
 * outside its door. `mode` is 'astar' (f = g + h) or 'greedy' (f = h).
 * Returns { pushes, expansions } with pushes a list of { from, to, dir },
 * or null when no solution was found inside the budget.
 */
function searchRoom(L, room, budget, mode) {
  const { w, h } = L;
  const cells = new Set(room.cells);
  cells.add(room.doorOutside);
  const goals = room.goals;
  const goalSet = new Set(goals);
  const dists = goals.map((g) => bfsDist(cells, g, w, h));
  const live = computeLive(cells, goals, w, h);
  const startBoxes = [...room.boxes].sort((a, b) => a - b);
  const startRegion = flood(cells, new Set(startBoxes), room.doorOutside, w, h);
  if (startRegion.length === 0) return null;
  const startRep = startRegion[0];
  const h0 = matchingHeuristic(startBoxes, dists, goals);
  if (!isFinite(h0)) return null;

  const nodes = new Map();
  const heap = new Heap();
  let counter = 0;
  const startKey = stateKey(startBoxes, startRep);
  nodes.set(startKey, {
    g: 0,
    h: h0,
    parent: null,
    push: null,
    boxes: startBoxes,
    regionRep: startRep,
  });
  heap.push({ f: h0, c: counter++, key: startKey, g: 0 });
  let expansions = 0;

  while (heap.size() > 0) {
    const top = heap.pop();
    const node = nodes.get(top.key);
    if (!node || node.g !== top.g) continue; // stale entry
    expansions++;
    totalExpansions++;
    if (expansions > budget || totalExpansions > GLOBAL_BUDGET) return null;
    if (node.boxes.every((b) => goalSet.has(b))) {
      const pushes = [];
      let cur = top.key;
      while (nodes.get(cur).parent !== null) {
        pushes.push(nodes.get(cur).push);
        cur = nodes.get(cur).parent;
      }
      pushes.reverse();
      return { pushes, expansions };
    }
    const boxSet = new Set(node.boxes);
    const region = flood(cells, boxSet, node.regionRep, w, h);
    const regionSet = new Set(region);
    for (const b of node.boxes) {
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di];
        const behind = nb(b, { dx: -d.dx, dy: -d.dy }, w, h);
        const target = nb(b, d, w, h);
        if (behind < 0 || target < 0) continue;
        if (!regionSet.has(behind)) continue;
        if (!cells.has(target) || boxSet.has(target)) continue;
        const nextBoxes = node.boxes.map((x) => (x === b ? target : x));
        nextBoxes.sort((a, b2) => a - b2);
        const nextBoxSet = new Set(nextBoxes);
        // Deadlock pruning.
        let dead = false;
        for (const nb2 of nextBoxes) {
          if (!live[nb2] && !goalSet.has(nb2)) {
            dead = true;
            break;
          }
        }
        if (dead) continue;
        if (hasDeadBlock(nextBoxes, nextBoxSet, cells, goalSet, w, h)) continue;
        const nextRegion = flood(cells, nextBoxSet, b, w, h);
        if (nextRegion.length === 0) continue;
        const nextRep = nextRegion[0];
        const nh = matchingHeuristic(nextBoxes, dists, goals);
        if (!isFinite(nh)) continue;
        const ng = node.g + 1;
        const nkey = stateKey(nextBoxes, nextRep);
        const prev = nodes.get(nkey);
        if (prev && prev.g <= ng) continue;
        nodes.set(nkey, {
          g: ng,
          h: nh,
          parent: top.key,
          push: { from: b, to: target, dir: di },
          boxes: nextBoxes,
          regionRep: nextRep,
        });
        heap.push({
          f: mode === 'astar' ? ng + nh : nh,
          c: counter++,
          key: nkey,
          g: ng,
        });
      }
    }
  }
  return null;
}

/** Turn a push list into LURD steps, walking the player between pushes. */
function pushesToMoves(L, room, startPlayer, startBoxes, pushes) {
  const { w, h } = L;
  const cells = new Set(room.cells);
  cells.add(room.doorOutside);
  let player = startPlayer;
  const boxes = new Set(startBoxes);
  let moves = '';
  for (const p of pushes) {
    const d = DIRS[p.dir];
    const behind = nb(p.from, { dx: -d.dx, dy: -d.dy }, w, h);
    const walk = walkPath(cells, boxes, player, behind, w, h);
    if (walk === null) return null;
    moves += walk;
    moves += d.ch;
    boxes.delete(p.from);
    boxes.add(p.to);
    player = p.from;
  }
  return { moves, player, boxes };
}

// ---------------------------------------------------------------------------
// Whole-level solving and entry point
// ---------------------------------------------------------------------------

/** Solve one room push-optimally (A*, greedy fallback). Returns the room's
 *  pushes or null. */
function solveOneRoom(L, room) {
  let res = searchRoom(L, room, ASTAR_BUDGET, 'astar');
  if (res) return res.pushes;
  res = searchRoom(L, room, GREEDY_BUDGET, 'greedy');
  if (res) return res.pushes;
  return null;
}

/** Solve a whole level by its rooms. Returns the LURD move string, or ''
 *  when a room could not be solved (an empty answer is valid and scores 0). */
function solveLevel(L) {
  const { w, h } = L;
  const rooms = segmentRooms(L);
  // Validate the segmentation: every box and goal belongs to exactly one
  // room, and each room's boxes match its goals. Otherwise treat the whole
  // level as a single room and let the same search have a go at it.
  let ok = rooms.length > 0;
  if (ok) {
    const seenBoxes = new Set();
    const seenGoals = new Set();
    for (const r of rooms) {
      if (r.boxes.length === 0 || r.boxes.length !== r.goals.length) ok = false;
      for (const b of r.boxes) {
        if (seenBoxes.has(b)) ok = false;
        seenBoxes.add(b);
      }
      for (const g of r.goals) {
        if (seenGoals.has(g)) ok = false;
        seenGoals.add(g);
      }
    }
    if (seenBoxes.size !== L.boxes.length) ok = false;
    let goalCount = 0;
    for (let i = 0; i < w * h; i++) if (L.goal[i]) goalCount++;
    if (seenGoals.size !== goalCount) ok = false;
  }
  if (!ok) {
    const allCells = new Set();
    for (let i = 0; i < w * h; i++) if (!L.wall[i]) allCells.add(i);
    const goals = [];
    for (let i = 0; i < w * h; i++) if (L.goal[i]) goals.push(i);
    const whole = {
      cells: allCells,
      door: L.player,
      doorOutside: L.player,
      boxes: [...L.boxes],
      goals,
    };
    const pushes = solveOneRoom(L, whole);
    if (!pushes) return '';
    const conv = pushesToMoves(L, whole, L.player, L.boxes, pushes);
    return conv ? conv.moves : '';
  }

  // Rooms in a fixed order: above rooms left to right, then below rooms.
  const hy = (L.player / w) | 0;
  rooms.sort((a, b) => {
    const aAbove = a.door < hy * w ? 0 : 1;
    const bAbove = b.door < hy * w ? 0 : 1;
    if (aAbove !== bAbove) return aAbove - bAbove;
    return a.door - b.door;
  });

  const allCells = new Set();
  for (let i = 0; i < w * h; i++) if (!L.wall[i]) allCells.add(i);
  let player = L.player;
  const boxes = new Set(L.boxes);
  let moves = '';
  for (const room of rooms) {
    // Walk along the hallway (and out of the previous room) to this door.
    const walk = walkPath(allCells, boxes, player, room.doorOutside, w, h);
    if (walk === null) return '';
    moves += walk;
    player = room.doorOutside;
    const pushes = solveOneRoom(L, room);
    if (!pushes) return '';
    const conv = pushesToMoves(L, room, player, room.boxes, pushes);
    if (!conv) return '';
    moves += conv.moves;
    for (const b of room.boxes) boxes.delete(b);
    for (const b of conv.boxes) boxes.add(b);
    player = conv.player;
  }
  return moves;
}

function main() {
  for (const id of IDS) {
    let moves = '';
    try {
      const text = readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8');
      const level = parseLevel(text);
      moves = solveLevel(level);
    } catch {
      moves = '';
    }
    const outDir = join(OUTPUT_DIR, id);
    mkdirSync(outDir, { recursive: true });
    writeFileSync(join(outDir, 'solution.txt'), moves + '\n', 'utf8');
  }
}

main();
