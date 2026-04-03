import time
import csv
import json
from locations import LOCATIONS
from scenarios import SCENARIOS
from app__ import build_solution

ALGORITHMS = ["MDP_Difference", "MDP", "Heuristic"]

RESULTS_FILE = "evaluation_results.csv"

FIELDNAMES = [
    "location",
    "scenario",
    "algorithm",
    "expected_total_sec",
    "expected_drive_sec",
    "expected_walk_sec",
    "expected_exit_sec",
    "expected_cross_sec",
    "success_probability",
    "runtime_ms",
    "chain_length",
    "status",
]


def run_evaluation():
    rows = []
    total = len(LOCATIONS) * len(SCENARIOS) * len(ALGORITHMS)
    done = 0

    for loc_key, loc in LOCATIONS.items():
        for scenario_key in SCENARIOS:
            if scenario_key == "Custom":
                continue

            for algo in ALGORITHMS:
                print(f"[{done + 1}/{total}] {loc_key} | {scenario_key} | {algo}")

                t_start = time.time()

                try:
                    result = build_solution(
                        start=loc["start"],
                        dest=loc["dest"],
                        ref=loc["ref"],
                        scenario_key=scenario_key,
                        algo_choice=algo,
                        exit_multiplier=1.0,
                        routing_mode="normal",
                        penalized_street=None,
                    )
                except Exception as e:
                    print(f"  ERROR: {e}")
                    rows.append(
                        {
                            "location": loc_key,
                            "scenario": scenario_key,
                            "algorithm": algo,
                            "status": f"error: {e}",
                            **{
                                f: None
                                for f in FIELDNAMES
                                if f
                                not in ("location", "scenario", "algorithm", "status")
                            },
                        }
                    )
                    done += 1
                    continue

                runtime_ms = (time.time() - t_start) * 1000

                if result["status"] != "success":
                    rows.append(
                        {
                            "location": loc_key,
                            "scenario": scenario_key,
                            "algorithm": algo,
                            "status": result.get("message", "failed"),
                            **{
                                f: None
                                for f in FIELDNAMES
                                if f
                                not in ("location", "scenario", "algorithm", "status")
                            },
                        }
                    )
                    done += 1
                    continue

                m = result["metrics"]

                rows.append(
                    {
                        "location": loc_key,
                        "scenario": scenario_key,
                        "algorithm": algo,
                        "expected_total_sec": round(m["expected_total_sec"], 2),
                        "expected_drive_sec": round(m["expected_drive_sec"], 2),
                        "expected_walk_sec": round(m["expected_walk_sec"], 2),
                        "expected_exit_sec": round(m["expected_exit_sec"], 2),
                        "expected_cross_sec": round(m["expected_cross_sec"], 2),
                        "success_probability": round(m["success_probability"], 4),
                        "runtime_ms": round(runtime_ms, 1),
                        "chain_length": len(result["chain"]),
                        "status": "success",
                    }
                )

                done += 1

    # save to CSV
    with open(RESULTS_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nDone. Results saved to {RESULTS_FILE}")
    return rows


def print_summary(rows):
    """
    Prints % improvement of MDP_Difference over Heuristic
    per scenario, averaged across 16 locations.
    """
    from collections import defaultdict

    # group by (scenario, algorithm)
    data = defaultdict(list)
    for r in rows:
        if r["status"] == "success" and r["expected_total_sec"] is not None:
            data[(r["scenario"], r["algorithm"])].append(r["expected_total_sec"])

    print("\n=== MDP_Difference vs Heuristic (avg expected_total_sec) ===\n")
    print(f"{'Scenario':<30} {'MDP':>10} {'Heuristic':>12} {'Improvement':>14}")
    print("-" * 70)

    for scenario_key in SCENARIOS:
        if scenario_key == "Custom":
            continue

        mdp_vals = data.get((scenario_key, "MDP_Difference"), [])
        heur_vals = data.get((scenario_key, "Heuristic"), [])

        if not mdp_vals or not heur_vals:
            continue

        mdp_avg = sum(mdp_vals) / len(mdp_vals)
        heur_avg = sum(heur_vals) / len(heur_vals)

        improvement = (heur_avg - mdp_avg) / heur_avg * 100

        print(
            f"{scenario_key:<30} {mdp_avg:>10.1f} {heur_avg:>12.1f} {improvement:>13.1f}%"
        )


if __name__ == "__main__":
    rows = run_evaluation()
    print_summary(rows)

## What you get when you run it
"""
A CSV file with this structure:
```
location | scenario | algorithm | expected_total_sec | expected_drive_sec | expected_walk_sec | expected_exit_sec | expected_cross_sec | success_probability | runtime_ms | chain_length | status
Set 1    | Delivery | MDP_Diff  | 342.1              | 180.3              | 48.2              | 95.4              | 18.2               | 0.96                | 1243.2     | 4            | success
Set 1    | Delivery | Heuristic | 498.3              | 175.1              | 112.4             | 182.6             | 28.2               | 0.94                | 890.1      | 3            | success
...
```

And a printed summary:
```
=== MDP_Difference vs Heuristic ===

Scenario                       MDP    Heuristic    Improvement
----------------------------------------------------------------------
Delivery (1-4m)               342.1       498.3          31.4%
Short Stay (10-40m)           412.3       489.1          15.7%
...
"""
