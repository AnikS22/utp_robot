# config/pipeline/ — local override copy of the sim repo's config

A verbatim copy of `~/unlocking-the-path/config/` with rover-laptop deltas, passed via
`python -m utp.runner.batch --config <this dir>`.

It exists because **CLAUDE.md forbids editing the simulation repo** ("Copy from it; never modify it
in place"), and both deltas below are properties of *this machine*, not of the experiment.

| file | delta | why |
|---|---|---|
| `methods.yaml` | added `matrices.keyfree: [passive, heuristic, ours_no_reasoning]` | the rows needing no VLM key, so the 47-mission benchmark runs offline |
| `detectors.yaml` | `owlv2`/`clip` `device: cuda:1`/`cuda:2` -> `cuda:0` | the workstation had 3 GPUs; this laptop has one, and a literal `cuda:1` raises at `.to()` |

**Re-sync when the sim repo's config changes** — this is a copy, not a symlink, and it will drift.
Diff with: `diff -r ~/unlocking-the-path/config config/pipeline`

## The `../missions` symlink

`Config._office_missions()` (`utp/common/config.py:209`) resolves the 47-mission answer key as
`config_dir.parent / "missions" / "office_building" / "missions.json"`. Because this override dir
lives at `config/pipeline/`, its parent is `config/` — so `config/missions` is a symlink to the sim
repo's `missions/`. Without it every office mission id fails with
`KeyError: unknown scene 'M0_open__00'`.

It is a symlink, not a copy, so the answer key has exactly one source of truth. It is gitignored:
a clone of this repo on a machine without the (private) sim repo would otherwise carry a dangling
link.
