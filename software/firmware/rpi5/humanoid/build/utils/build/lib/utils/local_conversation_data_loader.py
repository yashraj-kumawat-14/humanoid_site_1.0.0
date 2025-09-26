import os
import json
from utils.config_loader import get_config

def get_local_conversation_data():
    config = get_config()
    path = config["local_data_path"]
    if not path:
        raise RuntimeError("local_data_path not set. Please make sure it is validly exist in config.json")
    with open(path, "r") as f:
        return json.load(f) #dict[str, dict[str, str]]

