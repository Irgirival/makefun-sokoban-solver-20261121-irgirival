# makefun-sokoban-solver-20261121-irgirival

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Irgirival/makefun-sokoban-solver-20261121-irgirival/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Irgirival/makefun-sokoban-solver-20261121-irgirival/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for make.fun's sokoban-solver-01-run-20261121: one program answers a whole set of 5 levels in a single run.

## How it works

Each level is a hallway with 1–4 rooms off it. A box pushed out of a room's door can never be pushed back, so the rooms are independent problems joined only by the player's walk — and the level's push-optimal count is the sum of its rooms' optima. The solver therefore:

1. **Segments** each level into rooms (the hallway is the player's row; each room hangs off it through a single door).
2. **Solves every room push-optimally** with A* over push-states: min-cost box-to-goal matching as the heuristic, dead-square and 2×2-block deadlock pruning, and player-region canonicalisation (the player's exact square is replaced by its reachable region, so equivalent positions collapse to one state).
3. **Stitches** the rooms together with shortest walks along the hallway, converting each room's push list into LURD steps.

## Results

- Practice set: **5/5 levels at the push-optimal count** (13, 16, 29, 47, 60 pushes) — **score 1,000,000**.
- Also validated on freshly generated sets from the competition's own generator (see the test notes in the repo history): every level solved at its optimum.
- Deterministic: two runs write byte-identical answers (no clock, no randomness; work budgets are counted in expanded states, never in wall time). Whole-set runtime is a few seconds of CPU, far inside the 60-second wall budget.

## Run it

```sh
# the grader runs: node /bot/main.ts   (reads /input, writes /output)
# locally:
MAKEFUN_INPUT_DIR=path/to/input MAKEFUN_OUTPUT_DIR=path/to/output node main.ts
```

Node 22+, standard library only, no network, no dependencies.

---

<sub>Entered on [make.fun](https://make.fun/?ref=kfz6780b&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
