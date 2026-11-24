"""
Heterogeneous swarm task allocation: simulation + comparison.

8 robots (4 slow, 4 fast) must complete 3 randomly placed tasks, with no
central controller. Three strategies are compared:

  none    - every robot goes for the nearest task (no communication)
  comm    - robots hear neighbours within a radius and avoid tasks they
            have already claimed
  hetero  - like comm, but slow and fast robots differ in communication
            radius, avoidance penalty and distance preference

Metrics (all modes):
  time       - simulated seconds until every task is done (frames / FPS)
  conflicts  - frames where 2+ robots chased the same task (summed over
               frames, so it is "conflict-frames", not separate clashes)
  contacts   - frames where two robots were closer than COLLISION_DIST
  wasted     - times a robot dropped an unfinished target for another one

Usage:
  python swarm_sim.py                      # compare all modes, 30 runs each
  python swarm_sim.py --runs 100           # more runs, steadier averages
  python swarm_sim.py --mode comm --visual # watch one run in a window
"""

import argparse
import math
import random
import statistics

# --- SETTINGS ---
WIDTH, HEIGHT = 800, 600
NUM_AGENTS = 8
NUM_TASKS = 3
FPS = 10                 # simulated frames per second
MAX_FRAMES = 5000        # safety stop so a run can never loop forever
REACH_DIST = 10          # a task is done when a robot gets this close
COLLISION_DIST = 10      # "close contact" threshold

MODES = ("none", "comm", "hetero")

# --- COMMUNICATION SETTINGS (comm mode) ---
COMM_RADIUS = 80
COMM_PENALTY = 1000
COMM_NOISE = 30          # small randomness that helps break ties

# --- PER-TYPE SETTINGS (hetero mode; speeds are used in every mode) ---
ROBOT_TYPES = {
    "slow": dict(speed=1.2, comm_radius=60, penalty=1200, weight=1.3),
    "fast": dict(speed=2.2, comm_radius=120, penalty=300, weight=0.7),
}
HETERO_NOISE = 20

# --- COLORS (visual mode) ---
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)       # slow
RED = (255, 0, 0)        # fast
GREEN = (0, 200, 0)
GREY = (200, 200, 200)


def distance(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


class Task:
    def __init__(self, rng):
        self.x = rng.randint(50, WIDTH - 50)
        self.y = rng.randint(50, HEIGHT - 50)
        self.completed = False


class Agent:
    def __init__(self, x, y, kind, mode, rng):
        self.x = x
        self.y = y
        self.kind = kind          # "slow" or "fast"
        self.mode = mode
        self.rng = rng
        self.speed = ROBOT_TYPES[kind]["speed"]
        self.target = None
        self.wasted_steps = 0

        if mode == "hetero":
            p = ROBOT_TYPES[kind]
            self.comm_radius = p["comm_radius"]
            self.penalty = p["penalty"]
            self.weight = p["weight"]     # <1 prefers far tasks, >1 prefers near
            self.noise = HETERO_NOISE
        else:
            self.comm_radius = COMM_RADIUS
            self.penalty = COMM_PENALTY
            self.weight = 1.0
            self.noise = COMM_NOISE

    def choose_task(self, tasks, agents):
        # 1. Listen: which tasks have nearby robots already claimed?
        blocked = set()
        if self.mode != "none":
            for other in agents:
                if other is not self and other.target is not None:
                    if distance(self, other) < self.comm_radius:
                        blocked.add(other.target)

        # 2. Score every unfinished task; lowest cost wins.
        best_task = None
        best_cost = float("inf")
        for task in tasks:
            if task.completed:
                continue
            cost = distance(self, task) * self.weight
            if self.mode != "none":
                if task in blocked:
                    cost += self.penalty
                cost += self.rng.uniform(0, self.noise)
            if cost < best_cost:
                best_cost = cost
                best_task = task

        # 3. Count it as wasted effort if we abandoned an unfinished target.
        if (self.target is not None and not self.target.completed
                and best_task is not self.target):
            self.wasted_steps += 1
        self.target = best_task

    def move(self):
        if self.target is None:
            return
        dx = self.target.x - self.x
        dy = self.target.y - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > 1:
            # dx/dist and dy/dist are the direction; speed sets the step size
            self.x += self.speed * dx / dist
            self.y += self.speed * dy / dist

        if dist < REACH_DIST:
            self.target.completed = True
            self.target = None


def make_world(mode, seed):
    """Same seed -> same starting layout in every mode (fair comparison)."""
    layout_rng = random.Random(seed)
    decision_rng = random.Random(seed + 10_000)
    tasks = [Task(layout_rng) for _ in range(NUM_TASKS)]
    agents = []
    for i in range(NUM_AGENTS):
        x = layout_rng.randint(0, WIDTH)
        y = layout_rng.randint(0, HEIGHT)
        kind = "slow" if i < NUM_AGENTS // 2 else "fast"
        agents.append(Agent(x, y, kind, mode, decision_rng))
    return tasks, agents


def step(tasks, agents, stats):
    """Advance the simulation by one frame and update the metrics."""
    for agent in agents:
        agent.choose_task(tasks, agents)

    # conflicts: robots beyond the first that chase the same task
    counts = {}
    for agent in agents:
        if agent.target is not None:
            counts[agent.target] = counts.get(agent.target, 0) + 1
    stats["conflicts"] += sum(c - 1 for c in counts.values() if c > 1)

    # close contacts between robot pairs
    for i in range(len(agents)):
        for j in range(i + 1, len(agents)):
            if distance(agents[i], agents[j]) < COLLISION_DIST:
                stats["contacts"] += 1

    for agent in agents:
        agent.move()


def run_once(mode, seed):
    """Headless run (no window). Returns the metrics for one run."""
    tasks, agents = make_world(mode, seed)
    stats = {"conflicts": 0, "contacts": 0}
    frames = 0
    while frames < MAX_FRAMES and not all(t.completed for t in tasks):
        step(tasks, agents, stats)
        frames += 1
    return {
        "time": frames / FPS,
        "conflicts": stats["conflicts"],
        "contacts": stats["contacts"],
        "wasted": sum(a.wasted_steps for a in agents),
        "finished": all(t.completed for t in tasks),
    }


def compare(runs):
    """Run every mode `runs` times and print an average table."""
    print(f"\n{runs} runs per mode, {NUM_AGENTS} robots, {NUM_TASKS} tasks")
    print("(same starting layouts in every mode; values are mean +/- std)\n")
    header = f"{'mode':<8}{'time (s)':>16}{'conflicts':>16}{'contacts':>16}{'wasted':>14}"
    print(header)
    print("-" * len(header))

    for mode in MODES:
        results = [run_once(mode, seed) for seed in range(runs)]
        unfinished = sum(1 for r in results if not r["finished"])

        def cell(key, width, decimals=1):
            values = [r[key] for r in results]
            mean = statistics.mean(values)
            std = statistics.stdev(values) if len(values) > 1 else 0.0
            return f"{mean:.{decimals}f} +/- {std:.{decimals}f}".rjust(width)

        print(f"{mode:<8}{cell('time', 16)}{cell('conflicts', 16)}"
              f"{cell('contacts', 16)}{cell('wasted', 14)}")
        if unfinished:
            print(f"         (!) {unfinished} run(s) hit MAX_FRAMES")
    print()


def run_visual(mode, seed):
    """Watch one run in a Pygame window."""
    import pygame  # imported here so batch runs don't need a display

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"Swarm simulation - mode: {mode}")
    clock = pygame.time.Clock()

    tasks, agents = make_world(mode, seed)
    stats = {"conflicts": 0, "contacts": 0}
    frames = 0
    running = True

    while running and not all(t.completed for t in tasks):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        step(tasks, agents, stats)
        frames += 1

        screen.fill(CYAN)
        for task in tasks:
            if not task.completed:
                pygame.draw.circle(screen, GREEN, (int(task.x), int(task.y)), 10)
        for agent in agents:
            color = BLUE if agent.kind == "slow" else RED
            pos = (int(agent.x), int(agent.y))
            pygame.draw.circle(screen, color, pos, 6)
            if mode != "none":
                pygame.draw.circle(screen, GREY, pos, int(agent.comm_radius), 1)
        pygame.display.update()
        clock.tick(FPS)

    if all(t.completed for t in tasks):
        print(f"\nmode: {mode}  seed: {seed}")
        print(f"time: {frames / FPS:.1f} s")
        print(f"conflicts: {stats['conflicts']}")
        print(f"contacts: {stats['contacts']}")
        print(f"wasted: {sum(a.wasted_steps for a in agents)}")
        pygame.time.delay(1500)
    pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Swarm task allocation sim")
    parser.add_argument("--mode", choices=MODES, default="comm",
                        help="strategy for --visual (default: comm)")
    parser.add_argument("--visual", action="store_true",
                        help="watch one run in a window instead of comparing")
    parser.add_argument("--runs", type=int, default=30,
                        help="runs per mode for the comparison (default: 30)")
    parser.add_argument("--seed", type=int, default=0,
                        help="starting layout for --visual (default: 0)")
    args = parser.parse_args()

    if args.visual:
        run_visual(args.mode, args.seed)
    else:
        compare(args.runs)


if __name__ == "__main__":
    main()
