# maps/

Site maps produced by `docs/MAPPING.md`. Each map is two or three files:

| file | what it is | used by |
|---|---|---|
| `<name>.pgm` | occupancy grid image | `map_server`, via the `.yaml` |
| `<name>.yaml` | resolution, origin, thresholds | `ranger_nav.launch.py map:=maps/<name>.yaml` |
| `<name>.posegraph` | slam_toolbox's internal graph | resuming mapping, or `mode: localization` |

Save **both** the `.pgm`/`.yaml` pair and the `.posegraph`: the pair is what Nav2 loads, and the
posegraph is the only thing that lets you *continue* mapping later rather than starting over.

`.pgm` files are committed deliberately — a site map is a measurement, and a result that cannot be
reproduced against the map it was collected on is not reproducible. Record in `EXPERIMENT_LOG.md`
which map each mission run used, along with the corridor-width check from `MAPPING.md`.
