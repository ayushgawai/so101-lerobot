"""Log real-robot evaluation results from a scoresheet to Weights & Biases.

Reads a manually-scored eval scoresheet (Fixed + Random tabs) and pushes summary
metrics into the training run so success rate lives next to the loss curve.
Success is human-scored, so it is computed here from the raw Grasped?/Placed?
cells rather than Excel's cached formulas.

Usage (defaults match the ACT run):
    python scripts/log_eval_to_wandb.py                 # log to run 3n0nc42f
    python scripts/log_eval_to_wandb.py --dry-run       # print metrics, don't log
    python scripts/log_eval_to_wandb.py --xlsx SmolVLA_eval_scoresheet.xlsx \
        --project so101-smolvla --run-id <id> --checkpoint checkpoints/020000
"""

import argparse
import re
import zipfile
from xml.etree import ElementTree as ET

NS = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"

# Per-trial layout in each tab: header on row 14, trials on rows 15-64. The
# Random tab carries an extra "Gripper Pos" column, so its scoring columns all
# sit one to the right of the Fixed tab's.
FIRST_TRIAL_ROW = 15
LAST_TRIAL_ROW = 64
TAB_COLUMNS = {
    "Fixed": {"grasped": "C", "placed": "D", "failure": "E", "err": "F"},
    "Random": {"grasped": "D", "placed": "E", "failure": "F", "err": "G"},
}
FAILURE_MODES = ("no-grasp", "grasped-dropped", "wrong-placement", "other")


def _read_sheet_cells(xlsx_path, sheet_name):
    """Return {cell_ref: value_str} for one worksheet, resolving shared strings."""
    with zipfile.ZipFile(xlsx_path) as z:
        shared = []
        if "xl/sharedStrings.xml" in z.namelist():
            root = ET.fromstring(z.read("xl/sharedStrings.xml"))
            for si in root:
                shared.append("".join(t.text or "" for t in si.iter(NS + "t")))

        wb = ET.fromstring(z.read("xl/workbook.xml"))
        names = [s.get("name") for s in wb.iter(NS + "sheet")]
        if sheet_name not in names:
            raise ValueError(f"Sheet {sheet_name!r} not found; have {names}")
        sheet_files = sorted(
            n for n in z.namelist() if re.match(r"xl/worksheets/sheet\d+\.xml", n)
        )
        sheet_file = sheet_files[names.index(sheet_name)]

        cells = {}
        root = ET.fromstring(z.read(sheet_file))
        for c in root.iter(NS + "c"):
            if c.get("t") == "inlineStr":
                # openpyxl-written sheets store text inline instead of in sharedStrings.
                inline = c.find(NS + "is")
                if inline is None:
                    continue
                cells[c.get("r")] = "".join(t.text or "" for t in inline.iter(NS + "t"))
                continue
            v = c.find(NS + "v")
            if v is None or v.text is None:
                continue
            val = shared[int(v.text)] if c.get("t") == "s" else v.text
            cells[c.get("r")] = val
    return cells


def _yn(value):
    """True/False/None for a Y/N cell (case- and whitespace-insensitive)."""
    if value is None:
        return None
    s = str(value).strip().lower()
    if s in ("y", "yes", "1", "true"):
        return True
    if s in ("n", "no", "0", "false"):
        return False
    return None


def compute_metrics(xlsx_path, sheet_name):
    cells = _read_sheet_cells(xlsx_path, sheet_name)
    cols = TAB_COLUMNS[sheet_name]
    trials = 0
    grasped = 0
    successes = 0
    errors = []
    fails = {m: 0 for m in FAILURE_MODES}

    for row in range(FIRST_TRIAL_ROW, LAST_TRIAL_ROW + 1):
        g = _yn(cells.get(f"{cols['grasped']}{row}"))
        p = _yn(cells.get(f"{cols['placed']}{row}"))
        fail = cells.get(f"{cols['failure']}{row}")
        err = cells.get(f"{cols['err']}{row}")

        if g is None and p is None and not fail:
            continue  # trial not scored yet
        trials += 1
        if g:
            grasped += 1
        if g and p:
            successes += 1
            if err not in (None, ""):
                try:
                    errors.append(float(err))
                except ValueError:
                    pass
        if fail:
            key = str(fail).strip().lower()
            if key in fails:
                fails[key] += 1

    m = {
        "trials_scored": trials,
        "successes": successes,
        "success_rate": successes / trials if trials else 0.0,
        "grasp_rate": grasped / trials if trials else 0.0,
        "mean_placement_error_cm": sum(errors) / len(errors) if errors else None,
        "placement_mse_cm2": sum(e * e for e in errors) / len(errors) if errors else None,
    }
    for mode in FAILURE_MODES:
        m[f"fail_{mode.replace('-', '_')}"] = fails[mode]
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xlsx", default="ACT_eval_scoresheet.xlsx")
    ap.add_argument("--project", default="so101-act")
    ap.add_argument("--run-id", default="3n0nc42f",
                    help="Training run to resume and attach eval metrics to.")
    ap.add_argument("--entity", default=None)
    ap.add_argument("--checkpoint", default="checkpoints/030000",
                    help="Checkpoint label, stored on the metrics for reference.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Compute and print metrics without touching W&B.")
    args = ap.parse_args()

    results = {tab: compute_metrics(args.xlsx, tab) for tab in ("Fixed", "Random")}

    print(f"Checkpoint: {args.checkpoint}\n")
    for tab, m in results.items():
        print(f"[{tab}]  {m['trials_scored']}/50 scored")
        print(f"  success_rate : {m['success_rate']:.1%}  ({m['successes']} successes)")
        print(f"  grasp_rate   : {m['grasp_rate']:.1%}")
        mpe = m["mean_placement_error_cm"]
        print(f"  mean_err_cm  : {mpe:.2f}" if mpe is not None else "  mean_err_cm  : -")
        print(f"  failures     : " + ", ".join(
            f"{mode}={m['fail_' + mode.replace('-', '_')]}" for mode in FAILURE_MODES))
        print()

    total_scored = sum(m["trials_scored"] for m in results.values())
    if total_scored == 0:
        print("No trials scored yet — fill in the sheet before logging.")
        return
    if total_scored < 100:
        print(f"WARNING: only {total_scored}/100 trials scored; logging partial results.")

    if args.dry_run:
        print("Dry run — nothing sent to W&B.")
        return

    import wandb

    run = wandb.init(project=args.project, id=args.run_id, resume="must")
    for tab, m in results.items():
        prefix = f"eval/{tab.lower()}"
        for k, v in m.items():
            if v is not None:
                run.summary[f"{prefix}/{k}"] = v
    run.summary["eval/checkpoint"] = args.checkpoint

    # Side-by-side success-rate bar for the run dashboard.
    table = wandb.Table(
        columns=["scenario", "success_rate", "grasp_rate", "trials_scored"],
        data=[[tab.lower(), m["success_rate"], m["grasp_rate"], m["trials_scored"]]
              for tab, m in results.items()],
    )
    run.log({"eval/summary_table": table,
             "eval/success_rate_bar": wandb.plot.bar(
                 table, "scenario", "success_rate",
                 title=f"{args.project} success rate")})
    run.finish()
    print(f"\nLogged eval metrics to W&B run {args.run_id} in project {args.project}.")


if __name__ == "__main__":
    main()
