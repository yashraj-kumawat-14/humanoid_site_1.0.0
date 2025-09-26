import os
import json

def get_config():
    config_path = os.environ.get("HUMANOID_CONFIG")
    if not config_path:
        raise RuntimeError("HUMANOID_CONFIG not set. Please source setup.bash first.")
    with open(config_path, "r") as f:
        return json.load(f)

