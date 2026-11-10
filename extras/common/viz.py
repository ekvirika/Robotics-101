import os

import rerun as rr


def start_rerun(app_id: str) -> bool:
    mode = os.environ.get("VIEWER", "rerun")
    if mode == "none":
        return False
    if mode == "save":
        os.makedirs("out", exist_ok=True)
        rr.init(app_id)
        rr.save(f"out/{app_id}.rrd")
        return True
    rr.init(app_id, spawn=True)
    return True
