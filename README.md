[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/ytta90/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/ytta90/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Sokoban solver

A push-optimal Sokoban solver for make.fun's [Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty.

## Run

```sh
node main.ts
```

It reads the set from `/input/<id>/level.txt` (or `$MAKEFUN_INPUT_DIR`) and writes
`/output/<id>/solution.txt` (or `$MAKEFUN_OUTPUT_DIR`) for every instance: the
player's moves in `u`, `d`, `l`, `r`. Node 22, no dependencies, no network, no
clock reads — two runs write the same bytes. One run answers a whole set of 5
levels in well under a second.

## Design

A level is a hallway with independent rooms: a box pushed out of a room through
its door can never be pushed back, so each room is solved on its own and the
push counts add up to the level's optimum. Each room is solved with A\* over
pushes — a state is the boxes plus the player's reachable region, the cost is
pushes made, and the estimate is the sum over boxes of the fewest pushes that
box alone needs to reach a goal (precomputed by pulling boxes back from the
goals while ignoring the other boxes). That estimate never overstates the
pushes left, so the first solved state A\* pops is push-optimal. Two deadlocks
are pruned: a box on a square no pull can reach any goal from, and a frozen
2×2 block of walls and boxes holding a box off its goal. If A\* runs out of its
state budget a room is retried with greedy best-first search. All budgets are
counted in generated states, never wall-clock time, so the program is fully
deterministic.

## Results

- Practice set: **1,000,000** — all 5 instances solved in the push-optimal
  count (13, 16, 29, 47, 60 pushes), ~0.45 s.
- 8 freshly drawn sets from the kit's own generator (new levels, new seeds):
  **1,000,000 on every set**, every instance push-optimal, each set under 0.6 s.
- House baseline plain-astar 1.0.0 on the same sets: 200,000 (practice) and
  285,714 (fresh set 1).
