# Reproducible experiments

The pipeline runs simulations, exports per-run and aggregate results, and plots eight figures.

**Requirements:** Java 17+, Maven, Python 3.12. Run all commands from the `Java sim` project directory.

**Contents:** [Quick start](#quick-start) · [Configure a batch](#configure-a-batch) · [Units and formulas](#units-and-formulas) · [State time and attack outcomes](#state-time-and-attack-outcomes) · [Results and reproduction](#results-and-reproduction) · [Known limitations](#known-limitations)

## Quick start

```powershell
mvn test
mvn javafx:run "-Djavafx.args=-e configs/experiments/illustrative.yaml"
python -m pip install -r scripts/requirements.txt
python scripts/plot_results.py outputs/results/illustrative_reproducible/experiment_summary.csv
python -m unittest discover -s scripts -p "test_*.py"
```

`illustrative.yaml` defines 9 designs (3 experiment types × TURTLEBOT, DRONE, HYBRID) with 30 explicit parameter combinations and 3 repetitions, i.e. 90 simulations.

**How realistic are the inputs?**

- The `*_demo*.yaml` robot models are approximations from public datasheets, not measurements. Ground robots follow TurtleBot3 Waffle Pi class hardware; drones follow a Mavic Air class quadcopter. Each file header states its source and units.
- Station models, battery prices, cycle lives, and the electricity tariff are still **illustrative inputs**. Review them before reporting research results.
- Drones move 300 m per tick, so use a perimeter much longer than 300 m for drone and hybrid runs.

### Rules for batches

- Every batch needs a new `experimentId`. Existing batch directories are refused, including failed ones.
- The plotting script reads one batch summary and writes to `outputs/plots/` (or the directory given by `--output`).
- Plot file names are fixed and overwritten on re-run. To keep figures, archive them with their batch or use `--output outputs/plots/<experimentId>`.

| Figures | Files |
|---|---|
| 1–3 (patrol density) | `figure_1_patrol_density_turtlebot.png`, `figure_2_patrol_density_drone.png`, `figure_3_patrol_density_hybrid.png` |
| 4–6 (attack density) | `figure_4_attack_density_turtlebot.png`, `figure_5_attack_density_drone.png`, `figure_6_attack_density_hybrid.png` |
| 7–8 (TCO) | `figure_7_capture_vs_tco.png`, `figure_8_tco_vs_capture.png` |

Figures 7 and 8 use the same summary rows and statistics. Plotting never runs simulations.

## Configure a batch

Copy `src/main/resources/configs/experiments/illustrative.yaml` and supply approved values. Paths may be filesystem paths (relative to the working directory) or classpath resources.

### Command line

| Flag | Purpose |
|---|---|
| `-e <experiment.yaml>` | Run a batch |
| `-r <scenario-or-snapshot.yaml>` | Run a single simulation (the only mode with visualization) |

### Experiment document

Top-level fields: `experimentId`, `baseConfig`, `repetitions`, optional `energyPriceEurPerKwh`, and `designs`. An experiment tariff overrides the base-scenario tariff.

| Element | Fields |
|---|---|
| Design | unique `id`, `type`, `system`, list of explicit `points` |
| Point | unique `id`, `resourceScenario`, optional robot model/count list, optional charging-station list, optional `attackProbability` |

Omitted factors inherit the base scenario. An explicitly empty station list means no stations.

### Defining variants

- Define resource variants with the existing robot and station YAML models. Battery capacity and price may differ between variants; station models define slots and power.
- Hybrid points give both ground and drone counts.
- The runner rejects invalid compositions, attack sweeps that change anything except attack probability, and patrol sweeps that change model proportions or other settings within a resource scenario.
- Patrol and TCO designs keep attack probability fixed.

### Running a subset

The runner can execute a subset of designs during development. Plotting all eight figures requires:

- one design per type/system combination;
- at least two density points for each of at least two resource scenarios;
- at least two attack probabilities per system;
- at least two TCO configurations per system.

Plot validation rejects mixed batches, duplicate summaries, missing groups, differing duration/perimeter/patrol/attack-lifetime controls, and undefined capture points.

### Patrol algorithms

Batches support **AsyncPatrol only**. Preparation rejects PhasePatrol, CounterPhasePatrol, RingPatrol, EncounterPatrol, and unknown names before creating any output.

- The first three are movement placeholders.
- EncounterPatrol moves independently after initial deployment. Negotiation, redistribution on encounters, and rearm-policy execution are not connected to the simulation loop, so recording its runs as encounter-negotiation results would be misleading.
- Single-simulation strategy selection is unchanged.

### Plot input checks

- `status.txt` beside the summary must contain `COMPLETE`.
- Deviation probability, deviation duration, rearm policy, and charge threshold must be defined on every row and constant across the summary. Missing or differing values are rejected.
- Older completed summaries that lack some of these columns are still accepted, but a warning names the missing controls and states that comparability is not fully validated.
- Programmatic `generate(data)` calls work without a status file.

## Units and formulas

| Parameter | Unit or formula |
|---|---|
| Simulation tick | Exactly one minute |
| Simulation duration, attack duration, deviation duration | Integer minutes (ticks) |
| Perimeter length, robot positions, sensor ranges | m |
| Robot `maxSpeed` | m/min (numerically the maximum distance per tick) |
| Battery capacity and level, charge threshold | Wh |
| Movement `batteryConsumptionRate` | Wh/m; energy = path length × rate |
| LiDAR `lidarBatteryConsumptionRate` | Wh/scan |
| Robot `baselineConsumptionWhPerMinute` | Wh/min; default 0.01 preserves the former constant drain |
| Station `power` | W per occupied slot; charge per tick = power / 60 Wh |
| Robot/station `price`, battery `replacementCostEur` | EUR |
| `energyPriceEurPerKwh` | EUR/kWh |
| Battery `cycleLife` | Equivalent full discharge cycles |
| Legacy battery `averageBatteryLife` | Hours; kept as metadata, not used as cycle life |

### Charging and energy accounting

- Station power was converted from the old charge increments of 1 and 5 to 60 W and 300 W, which preserves the 1 Wh and 5 Wh added per tick. This is a compatibility conversion, not a hardware claim.
- Every occupied slot receives the full configured power; changing the slot count does not divide it. The existing queue-selection heuristic and charging timing are unchanged.
- Charged and discharged energy are accumulated from clamped battery-level changes. Empty batteries add no phantom discharge, and a nearly full battery accepts only its remaining capacity.
- Batteries start full, and that initial energy is not billed.
- Charging efficiency is 1. Grid overhead, charger idle consumption, and calendar aging are not modeled.

### Battery wear

For each robot:

```text
equivalentFullDischargeCycles = cumulativeActualDischargedWh / batteryCapacityWh
batteryWearEur = replacementCostEur * equivalentFullDischargeCycles / cycleLife
```

### Total cost of ownership

```text
robotAcquisitionEur   = sum(robot model price × count)
stationAcquisitionEur = sum(station purchase price)
energyEur             = sum(actualChargedWh) / 1000 * energyPriceEurPerKwh
batteryReplacementEur = sum(robot batteryWearEur)
TCO = robotAcquisitionEur + stationAcquisitionEur + energyEur + batteryReplacementEur
```

- Wear is fractional depreciation, so it accrues before any whole battery is replaced.
- TCO designs require explicit tariffs, battery prices, and cycle lives. Other runs may omit them; unavailable cost terms are exported as empty fields.
- TCO excludes maintenance, staffing, financing, and non-battery replacements.

## State time and attack outcomes

### Robot states

Each robot contributes exactly one state count (one minute) per tick.

- Accounting uses the state reached during the update. A tick in which charging completes is counted as `CHARGING`, even if the robot switches to patrol for its next tick.
- Acquiring a charging slot counts as entering `CHARGING`; energy flows from the following update. This preserves the existing control flow.
- States: `PATROLLING`, `GOING_TO_PATROL`, `GOING_TO_CHARGE`, `WAITING_FOR_CHARGE`, `CHARGING`, `ERROR`.
- `WAITING_FOR_CHARGE` marks an unavailable charging slot or service, as distinct from travel.
- `ERROR` currently means battery exhaustion only. Configured hardware failure probabilities are not executed by the model, and no new failure process was introduced.

Every run checks that each robot's state counts equal the elapsed duration, so system state counts equal robot count × duration.

### System activity

```text
activityCoefficient = sum(robot PATROLLING minutes) / sum(robot non-PATROLLING minutes)
```

- It is a ratio of sums, not a mean of per-robot ratios, and it can exceed one.
- It is undefined when there is no non-patrol time.
- Figures label curves by resource scenario and annotate the mean measured activity at each point.

### Attack outcomes

Ticks run at times `0` through `duration - 1`; the terminal boundary is time `duration`. At that boundary:

- active attacks that have expired, or have no configured duration, are marked missed;
- intercepted attacks stay intercepted, including durationless ones;
- unexpired active attacks stay unresolved and are excluded from capture statistics.

```text
captureCoefficient = intercepted / (intercepted + missed)
```

- With no resolved attacks, capture is undefined. Undefined values are empty CSV fields, never zero percent.
- Detection time is the delay from creation to interception, in minutes, among intercepted attacks. It is undefined if nothing was intercepted.
- `detection_fraction_generated` is intercepted/generated and differs from resolved-attack capture.
- Legacy `successRate` keeps intercepted/generated for existing APIs and exports. Placeholder response-time metrics are not in the experiment schema.

`PoissonAttack.lambda` is a Bernoulli probability in [0, 1] per one-minute tick, not a conventional Poisson rate. At most one attack is generated per tick, so the expected number over T ticks is T × lambda. No attack or patrol algorithm is replaced by this framework.

## Results and reproduction

### Output files

Each `outputs/results/<experimentId>/` directory contains:

| File | Content |
|---|---|
| `experiment_runs.csv` | One row per simulation: identifiers, seeds, model/count and resource configuration, physical parameters, outcomes, costs, state totals, config path and hash |
| `experiment_summary.csv` | One row per parameter combination: total and valid run counts, means, and population SDs for each metric |
| `experiment_robot_states.csv` | One row per robot per run: time in each state, battery throughput, equivalent discharge cycles, wear cost |
| `configs/<designId>/<pointId>/run-<index>.yaml` | Resolved scenario snapshots with robot/station models, cost settings, and the actual run seed |
| `experiment.yaml`, `runtime.txt`, `status.txt` | Plan, runtime and statistical assumptions, batch completion status (the plotting script rejects incomplete or failed batches) |
| `implementation.zip` | Source, model YAML, plotting scripts, documentation, and Maven configuration (when launched from the project source directory) |

Archive `implementation.zip` with the results. If you launch only a packaged binary elsewhere, keep that binary and its source revision separately.

### Seeds

- `runSeed = simulation.seed + repetitionIndex`, with the index starting at zero. Only the YAML master seed is used.
- Equal repetition indices reuse the same seed across configurations.
- The simulator shares one random generator between behaviors, so equal seeds do **not** guarantee identical attack scenarios when configurations consume random numbers differently. Do not describe these runs as using aligned common random streams.

### Reproducing a run

```powershell
mvn javafx:run "-Djavafx.args=-r outputs/results/illustrative_reproducible/configs/tco_turtlebot/p0/run-1.yaml"
```

Snapshot loading from the filesystem uses the embedded model definitions, not later classpath revisions. Reproduction still requires the same implementation; the source archive and pinned Maven/Python dependencies provide that provenance.

### Statistics

- Aggregation uses per-run metrics, never pooled event counts or per-robot ratios.
- Undefined values are excluded separately for each metric's mean and SD. Total and valid run counts stay visible.
- SD is the population SD (divides by the number of defined observations), following the project's existing convention. One defined observation gives SD 0.
- SD error bars are not confidence intervals. The TCO figures show both capture and cost variation on their axes.

### Execution constraints

- The static world and random provider require sequential runs.
- World construction resets static robot, station, and perimeter state; IDs and random seeds reset at the start of each run.
- A small coordinate-scale floating-point tolerance in movement validation stops rounded positions from falsely exceeding the configured speed.

## Known limitations

The pipeline checks preserve current simulation behavior. These pre-existing issues still need separate model validation because they can affect physical interpretation:

- abandoned or exhausted robots remaining in charging queues;
- exact LiDAR distance checks after cell lookup;
- scans and movement that exceed available energy;
- charging-selection units and multi-slot waiting estimates;
- circular Async patrol direction and section allocation.

Exported state and energy totals alone cannot attribute a run's battery-exhaustion time to a specific cause. None of these issues are corrected by the experiment/export validation changes. Encounter negotiation and the placeholder patrol/attack algorithms also remain separate feature work.
