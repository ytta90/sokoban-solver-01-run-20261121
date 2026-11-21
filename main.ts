// A push-optimal Sokoban solver for make.fun's sokoban-solver-01.
//
// How it works. A level is a hallway with independent rooms: a box pushed out
// of a room through its door can never be pushed back, so each room is solved
// on its own and the push counts add up to the level's optimum. Each room is
// solved with A* over pushes: a state is the boxes plus the player's reachable
// region (named by its lowest square), the cost is pushes made, and the
// estimate is the sum over boxes of the fewest pushes that box alone needs to
// reach a goal, precomputed by pulling boxes back from the goals while
// ignoring the other boxes. That estimate never overstates the pushes left
// (one push moves one box by one step of its own shortest route), so the first
// solved state A* pops is push-optimal. Two deadlocks are pruned: a box on a
// square no pull can reach any goal from, and a frozen 2x2 block of walls and
// boxes holding a box off its goal. When A* runs out of its state budget the
// room is retried with greedy best-first search, which still usually solves
// it, just not provably optimally. Everything is budgeted in generated states,
// never wall-clock time, so two runs write the same bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets, counted in generated states (never the clock).
const ROOM_ASTAR_STATES = 400_000;
const ROOM_GREEDY_STATES = 250_000;
const WHOLE_ASTAR_STATES = 300_000;
const TOTAL_STATES = 3_000_000;
let spent = 0;

const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

interface Board {
  w: number;
  h: number;
  size: number;
  wall: Uint8Array;
  goals: number[];
  goalSet: Uint8Array;
  boxes: number[];
  player: number;
}

interface Room {
  boxes: number[];
  goals: number[];
}

/** Parse XSB text onto a board padded with one wall cell all round, so every neighbour lookup stays in bounds. */
function parseLevel(text: string): Board {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w0 = Math.max(...rows.map((r) => r.length));
  const w = w0 + 2;
  const h = rows.length + 2;
  const size = w * h;
  const wall = new Uint8Array(size).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y0) => {
    for (let x0 = 0; x0 < row.length; x0++) {
      const ch = row[x0] as string;
      const i = (y0 + 1) * w + (x0 + 1);
      if (ch === '#') continue;
      wall[i] = 0;
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  return { w, h, size, wall, goals, goalSet, boxes, player };
}

/**
 * For every square, the fewest pushes one box alone needs to reach a goal
 * (-1 when no route exists), found by pulling boxes back from the goals: a
 * box on y pushed along d lands on y + d, so from a square x at distance k the
 * square y = x - d is at distance k + 1 when the player can stand at y - d.
 */
function reverseDist(wall: Uint8Array, goals: number[], w: number, size: number): Int32Array {
  const dist = new Int32Array(size).fill(-1);
  const queue: number[] = [];
  let head = 0;
  for (const g of goals) {
    dist[g] = 0;
    queue.push(g);
  }
  while (head < queue.length) {
    const x = queue[head++] as number;
    for (const d of DIRS) {
      const dd = d.dy * w + d.dx;
      const y = x - dd;
      if (y < 0 || y >= size) continue;
      if (!wall[y] && !wall[y - dd] && dist[y] < 0) {
        dist[y] = dist[x] as number + 1;
        queue.push(y);
      }
    }
  }
  return dist;
}

/** Flood the squares the player reaches from `from` around walls and boxes; returns the lowest one. */
function floodLow(
  wall: Uint8Array,
  occ: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
  w: number,
): number {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n] as number;
    if (p < low) low = p;
    for (const d of DIRS) {
      const q = p + d.dy * w + d.dx;
      if (!wall[q] && !occ[q] && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block of walls and boxes holding a box off its goal. */
function frozenAt(wall: Uint8Array, occ: Uint8Array, goalSet: Uint8Array, t: number, w: number): boolean {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occ[a]) || !(wall[b] || occ[b]) || !(wall[c] || occ[c]) || !(wall[d] || occ[d])) continue;
    if ((occ[a] && !goalSet[a]) || (occ[b] && !goalSet[b]) || (occ[c] && !goalSet[c]) || (occ[d] && !goalSet[d])) {
      return true;
    }
  }
  return false;
}

/** The shortest walk from `from` to `to` around walls and boxes, or null when none reaches. */
function walkTo(wall: Uint8Array, occ: Uint8Array, from: number, to: number, w: number, size: number): string | null {
  if (from === to) return '';
  const prev = new Int32Array(size).fill(-1);
  const how = new Uint8Array(size);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi] as number;
    if (p === to) break;
    for (let k = 0; k < 4; k++) {
      const d = DIRS[k] as { dx: number; dy: number; ch: string };
      const q = p + d.dy * w + d.dx;
      if (wall[q] || occ[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = k;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return null;
  let s = '';
  for (let p = to; p !== from; p = prev[p] as number) s = (DIRS[how[p] as number] as { ch: string }).ch + s;
  return s;
}

/**
 * Push-optimal (A*) or greedy search over pushes on one board: `wall` holds the
 * walls plus every box that is not this room's as a wall, `goals` this room's
 * goals, `boxes` this room's boxes, `dist` the pull-back distances on `wall`.
 * Returns the pushes as [box's old square, direction index] from first to
 * last, or null when the state budget runs out or the room cannot be solved.
 */
function searchPushes(
  w: number,
  size: number,
  wall: Uint8Array,
  goals: number[],
  boxes: number[],
  player: number,
  dist: Int32Array,
  maxStates: number,
  greedy: boolean,
): [number, number][] | null {
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const n = boxes.length;
  const occ = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let tick = 0;
  const keyOf = (bs: number[], low: number): number => {
    let k = low;
    for (const b of bs) k = k * size + b;
    return k;
  };
  const hOf = (bs: number[]): number => {
    let h = 0;
    for (const b of bs) h += dist[b] as number;
    return h;
  };
  const solved = (bs: number[]): boolean => {
    for (const b of bs) if (!goalSet[b]) return false;
    return true;
  };

  // The states, grown as needed: their boxes, pushes made, estimate, parent,
  // the push that made them, the player's square after it, and their key.
  let cap = 4096;
  let boxesFlat = new Int32Array(cap * n);
  let gScore = new Int32Array(cap);
  let parent = new Int32Array(cap);
  let pushFrom = new Int32Array(cap);
  let pushDir = new Uint8Array(cap);
  let after = new Int32Array(cap);
  let skey = new Float64Array(cap);
  let count = 0;
  const growStates = (): void => {
    const nc = cap * 2;
    const nbf = new Int32Array(nc * n);
    nbf.set(boxesFlat);
    boxesFlat = nbf;
    const ng = new Int32Array(nc);
    ng.set(gScore);
    gScore = ng;
    const np = new Int32Array(nc);
    np.set(parent);
    parent = np;
    const nf = new Int32Array(nc);
    nf.set(pushFrom);
    pushFrom = nf;
    const nd = new Uint8Array(nc);
    nd.set(pushDir);
    pushDir = nd;
    const na = new Int32Array(nc);
    na.set(after);
    after = na;
    const nk = new Float64Array(nc);
    nk.set(skey);
    skey = nk;
    cap = nc;
  };
  const addState = (bs: number[], g: number, par: number, from: number, dir: number, pl: number, key: number): number => {
    if (count === cap) growStates();
    const s = count++;
    boxesFlat.set(bs, s * n);
    gScore[s] = g;
    parent[s] = par;
    pushFrom[s] = from;
    pushDir[s] = dir;
    after[s] = pl;
    skey[s] = key;
    return s;
  };
  const stateBoxes = (s: number): number[] => Array.from(boxesFlat.subarray(s * n, s * n + n));

  // A binary heap on (f, h, state): lowest f first, then lowest estimate,
  // then the state made first, so the order is fixed from run to run.
  let hcap = 4096;
  let hf = new Int32Array(hcap);
  let hh = new Int32Array(hcap);
  let hs = new Int32Array(hcap);
  let hlen = 0;
  const growHeap = (): void => {
    const nc = hcap * 2;
    const nf = new Int32Array(nc);
    nf.set(hf);
    hf = nf;
    const nh = new Int32Array(nc);
    nh.set(hh);
    hh = nh;
    const ns = new Int32Array(nc);
    ns.set(hs);
    hs = ns;
    hcap = nc;
  };
  const hless = (i: number, j: number): boolean =>
    hf[i] !== hf[j] ? (hf[i] as number) < (hf[j] as number)
    : hh[i] !== hh[j] ? (hh[i] as number) < (hh[j] as number)
    : (hs[i] as number) < (hs[j] as number);
  const hswap = (i: number, j: number): void => {
    const tf = hf[i] as number;
    const th = hh[i] as number;
    const ts = hs[i] as number;
    hf[i] = hf[j] as number;
    hh[i] = hh[j] as number;
    hs[i] = hs[j] as number;
    hf[j] = tf;
    hh[j] = th;
    hs[j] = ts;
  };
  const hpush = (f: number, h: number, s: number): void => {
    if (hlen === hcap) growHeap();
    let i = hlen++;
    hf[i] = f;
    hh[i] = h;
    hs[i] = s;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!hless(i, p)) break;
      hswap(i, p);
      i = p;
    }
  };
  const hpop = (): [number, number, number] => {
    const f = hf[0] as number;
    const h = hh[0] as number;
    const s = hs[0] as number;
    const lf = hf[--hlen] as number;
    const lh = hh[hlen] as number;
    const ls = hs[hlen] as number;
    if (hlen > 0) {
      hf[0] = lf;
      hh[0] = lh;
      hs[0] = ls;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= hlen) break;
        if (c + 1 < hlen && hless(c + 1, c)) c++;
        if (!hless(c, i)) break;
        hswap(i, c);
        i = c;
      }
    }
    return [f, h, s];
  };

  // A box frozen from the start can never be solved.
  for (const b of boxes) occ[b] = 1;
  for (const b of boxes) {
    if (frozenAt(wall, occ, goalSet, b, w)) {
      for (const x of boxes) occ[x] = 0;
      return null;
    }
  }
  const low0 = floodLow(wall, occ, player, seen, ++tick, stack, w);
  for (const b of boxes) occ[b] = 0;
  if (solved(boxes)) return [];
  const h0 = hOf(boxes);
  const best = new Map<number, number>();
  const k0 = keyOf(boxes, low0);
  best.set(k0, 0);
  addState(boxes, 0, -1, -1, 0, player, k0);
  hpush(greedy ? h0 : h0, h0, 0);

  while (hlen > 0) {
    const [, , s] = hpop();
    if (best.get(skey[s] as number) !== gScore[s]) continue; // a better route to this state was found
    const bs = stateBoxes(s);
    if (solved(bs)) {
      const pushes: [number, number][] = [];
      for (let x = s; parent[x] as number >= 0; x = parent[x] as number) {
        pushes.push([pushFrom[x] as number, pushDir[x] as number]);
      }
      pushes.reverse();
      return pushes;
    }
    for (const b of bs) occ[b] = 1;
    const stamp = ++tick;
    floodLow(wall, occ, after[s] as number, seen, stamp, stack, w);
    const g = gScore[s] as number;
    for (let bi = 0; bi < n; bi++) {
      const b = bs[bi] as number;
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k] as { dx: number; dy: number; ch: string };
        const dd = d.dy * w + d.dx;
        const t = b + dd;
        const ps = b - dd;
        if (seen[ps] !== stamp) continue;
        if (wall[t] || occ[t] || dist[t] < 0) continue;
        occ[b] = 0;
        occ[t] = 1;
        if (frozenAt(wall, occ, goalSet, t, w)) {
          occ[t] = 0;
          occ[b] = 1;
          continue;
        }
        const nb = bs.slice();
        nb[bi] = t;
        nb.sort((a, b2) => a - b2);
        const low = floodLow(wall, occ, b, mark, ++tick, stack, w);
        const key = keyOf(nb, low);
        const g2 = g + 1;
        const prevG = best.get(key);
        if (prevG !== undefined && prevG <= g2) {
          occ[t] = 0;
          occ[b] = 1;
          continue;
        }
        if (count >= maxStates || spent >= TOTAL_STATES) {
          for (const x of bs) occ[x] = 0;
          return null;
        }
        spent++;
        best.set(key, g2);
        const h2 = hOf(nb);
        const ns = addState(nb, g2, s, b, k, b, key);
        hpush(greedy ? h2 : g2 + h2, h2, ns);
        occ[t] = 0;
        occ[b] = 1;
      }
    }
    for (const b of bs) occ[b] = 0;
  }
  return null;
}

/** Expand a list of pushes into the player's moves, walking to each push square first. */
function expandPushes(
  wall: Uint8Array,
  boxes: number[],
  player: number,
  pushes: [number, number][],
  w: number,
  size: number,
): string | null {
  const occ = new Uint8Array(size);
  const cur = boxes.slice();
  let p = player;
  let moves = '';
  for (const [from, dir] of pushes) {
    const d = DIRS[dir] as { dx: number; dy: number; ch: string };
    const dd = d.dy * w + d.dx;
    for (const b of cur) occ[b] = 1;
    const walk = walkTo(wall, occ, p, from - dd, w, size);
    for (const b of cur) occ[b] = 0;
    if (walk === null) return null;
    moves += walk + d.ch;
    const i = cur.indexOf(from);
    cur[i] = from + dd;
    cur.sort((a, b) => a - b);
    p = from;
  }
  return moves;
}

/** Play moves on a board, returning the boxes and player they leave. */
function applyMoves(
  wall: Uint8Array,
  boxes: number[],
  player: number,
  moves: string,
  w: number,
): { boxes: number[]; player: number } {
  const cur = boxes.slice();
  const occ = new Uint8Array(wall.length);
  for (const b of cur) occ[b] = 1;
  let p = player;
  for (const ch of moves) {
    const d = DIRS.find((x) => x.ch === ch) as { dx: number; dy: number; ch: string };
    const dd = d.dy * w + d.dx;
    const q = p + dd;
    if (occ[q]) {
      occ[q] = 0;
      occ[q + dd] = 1;
      const i = cur.indexOf(q);
      cur[i] = q + dd;
    }
    p = q;
  }
  cur.sort((a, b) => a - b);
  return { boxes: cur, player: p };
}

/**
 * Cut the level into rooms: the hallway row's dead squares (floor no pull can
 * reach a goal from) split the floor, and each part left holding boxes is a
 * room. Null when a box starts in the hallway or a part's boxes and goals do
 * not match.
 */
function roomsOf(level: Board): Room[] | null {
  const { w, size, wall, goals, boxes, player } = level;
  const dist = reverseDist(wall, goals, w, size);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(size);
  for (let x = 0; x < w; x++) {
    const i = row * w + x;
    if (!wall[i] && dist[i] < 0) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(size).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < size; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      const p = squares[i] as number;
      for (const d of DIRS) {
        const q = p + d.dy * w + d.dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [] });
  }
  for (const b of boxes) {
    if (part[b] < 0) return null;
    (rooms[part[b] as number] as Room).boxes.push(b);
  }
  for (const g of goals) {
    if (part[g] < 0) return null;
    (rooms[part[g] as number] as Room).goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Solve one level's moves, room by room, or '' when any room cannot be solved. */
function solveLevel(text: string): string {
  const level = parseLevel(text);
  const rooms = roomsOf(level);
  if (rooms === null) {
    // No room split: try the whole level as one search, then give up.
    const dist = reverseDist(level.wall, level.goals, level.w, level.size);
    if (level.boxes.every((b) => dist[b] >= 0)) {
      const pushes =
        searchPushes(level.w, level.size, level.wall, level.goals, level.boxes, level.player, dist, WHOLE_ASTAR_STATES, false) ??
        searchPushes(level.w, level.size, level.wall, level.goals, level.boxes, level.player, dist, WHOLE_ASTAR_STATES, true);
      if (pushes !== null) {
        const m = expandPushes(level.wall, level.boxes, level.player, pushes, level.w, level.size);
        if (m !== null) return m;
      }
    }
    return '';
  }
  let curBoxes = level.boxes.slice();
  let curPlayer = level.player;
  let all = '';
  const done = new Set<Room>();
  for (const room of rooms) {
    // The other rooms' boxes stand as walls (on their goals once done).
    const wall2 = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of done.has(other) ? other.goals : other.boxes) wall2[b] = 1;
    }
    const dist = reverseDist(wall2, room.goals, level.w, level.size);
    let pushes: [number, number][] | null = null;
    if (room.boxes.every((b) => dist[b] >= 0)) {
      pushes = searchPushes(level.w, level.size, wall2, room.goals, room.boxes, curPlayer, dist, ROOM_ASTAR_STATES, false);
      if (pushes === null) {
        pushes = searchPushes(level.w, level.size, wall2, room.goals, room.boxes, curPlayer, dist, ROOM_GREEDY_STATES, true);
      }
    }
    if (pushes === null) return '';
    const m = expandPushes(level.wall, curBoxes, curPlayer, pushes, level.w, level.size);
    if (m === null) return '';
    all += m;
    const afterState = applyMoves(level.wall, curBoxes, curPlayer, m, level.w);
    curBoxes = afterState.boxes;
    curPlayer = afterState.player;
    done.add(room);
  }
  return all;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
