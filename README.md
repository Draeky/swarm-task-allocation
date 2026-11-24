# Heterogeneous Swarm Task Allocation (Python + Pygame)

A simulation of 8 robots (4 slow, 4 fast) completing 3 randomly placed tasks
with **no central controller**. Each robot decides for itself which task to
chase, and the project compares how much local communication helps.

## Strategies compared

| Mode | How a robot chooses a task |
|---|---|
| `none` | Goes for the nearest unfinished task. |
| `comm` | Hears neighbours within a radius (80 px) and avoids tasks they already claimed (distance penalty). |
| `hetero` | Same as `comm`, but slow and fast robots differ in communication radius, avoidance penalty and distance preference. |

## Metrics

- **Time:** simulated seconds until all tasks are complete.
- **Conflicts:** frames in which two or more robots chased the same task (summed over frames).
- **Contacts:** frames in which two robots were closer than 10 px.
- **Wasted:** times a robot dropped an unfinished target for another one.

## Run it

```bash
pip install pygame          # only needed for --visual
python swarm_sim.py                       # compare all modes (30 runs each)
python swarm_sim.py --runs 100            # more runs, steadier averages
python swarm_sim.py --mode comm --visual --seed 2   # watch one run
```

Every run uses a fixed seed, so results are reproducible, and all three modes
start from the same layouts, so the comparison is fair.

## Results (30 seeded layouts per mode, mean +/- std)

| Mode | Time (s) | Conflicts | Contacts | Wasted |
|---|---|---|---|---|
| none | 10.6 +/- 4.2 | 658.8 +/- 277.8 | 7.6 +/- 16.8 | 0.0 +/- 0.0 |
| comm | 10.9 +/- 3.8 | 662.5 +/- 251.5 | 5.7 +/- 11.3 | 21.3 +/- 35.3 |
| hetero | 12.3 +/- 4.1 | 737.9 +/- 264.6 | 9.2 +/- 31.2 | 22.3 +/- 26.4 |

## What the results show

- **Local communication gave no clear speed-up.** Completion time is driven
  mostly by how far the robots start from the tasks, and the run-to-run spread
  is larger than the differences between modes.
- **It changes behaviour, not totals.** On some layouts communication removed
  close contacts entirely (for example seed 2: 48 contacts without it, 0 with
  it), but it also made robots switch targets more often (wasted effort).
- **The heterogeneous setting was slightly slower** in this configuration, so
  the chosen radii, penalties and weights are probably not tuned well.

## Limitations

- With 8 robots and only 3 tasks, at least 5 robots are always chasing a
  duplicate target, so the conflict count has a high floor and cannot show
  large improvements. More tasks than robots would test communication better.
- Conflicts are counted per frame, not as separate events.
- Parameters were set by hand and not tuned.

## Ideas for next steps

- Count only conflicts above the unavoidable minimum, or test with more tasks.
- Distance-based bidding (auction) to resolve conflicts inside the radius.
- Port the agents to ROS 2 nodes with task allocation over topics.
