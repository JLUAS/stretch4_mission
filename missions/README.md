# Local missions

The web UI writes site-specific mission YAML files here and lidar reference scans under `missions/scans/`.
They may expose map paths and physical locations, so Git ignores them by default.

Use `sim_missions/` for portable simulation examples. Add a real mission to version control only after reviewing
and sanitizing its map path, goals, and scan data.

