# Architecture decision records

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-edge-gpu-split.md) | Pi 5 edge / laptop GPU split; no YOLO on the robot | accepted |
| [0002](0002-capture-timestamp-contract.md) | Capture timestamp is the matching key; GPU never rewrites it | accepted |
| [0003](0003-distance-weighted-fusion.md) | Distance/quality-aware two-view information fusion | accepted |

New ADRs should record context, decision, consequences and the alternatives
that were rejected. Decisions that change a topic contract also require a
contract review against `jcheng-interfaces`.
