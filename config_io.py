"""Safe partial updates to config.json.

Every tuner/test tool here reads the whole config at startup and keeps it in
memory. Dumping that copy back on save silently reverts anything else that was
edited in config.json while the tool was open — this is what kept resetting the
velocities and the MQTT port back to stale values.

So: never write back the startup copy. Save through update(), which re-reads
from disk and touches only the keys you name.
"""
import json

CONFIG_FILE = "config.json"


def update(section, values, path=CONFIG_FILE):
    """Merge `values` into cfg[section] on disk and write it back.

    Every key other than the ones in `values` keeps whatever is on disk *right
    now*, not what this process read at startup. Returns the saved config.
    """
    with open(path, "r") as f:
        cfg = json.load(f)
    cfg.setdefault(section, {}).update(values)
    with open(path, "w") as f:
        json.dump(cfg, f, indent=4)
    return cfg
