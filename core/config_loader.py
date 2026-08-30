# Loads and parses config files from the config/ directory.
# Pure Python -- no Qt imports allowed in core/.

import json
import os

CONFIG_DIR = "config"

def load_car_config():
    path = os.path.join(CONFIG_DIR, "car.json")
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None

def get_setup_parameters():
    config = load_car_config()
    if not config:
        return {}
    return config.get("setup_parameters", {})