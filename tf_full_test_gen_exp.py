#!/usr/bin/env python3
import doepy
import pandas as pd
from doepy import build
import argparse
import pathlib
import json


def create_experiment(objects, distances, pans, tilts):
    pd_list = []
    for object in objects:
        object_experiment = build.full_fact(
            {'Distancia (m)':   distances,
            'Pan (deg)':        pans,
            'Tilt (deg)':       tilts,}
        )
        object_experiment["Object"] = object[0]
        object_experiment["Object path"] = object[1]
        pd_list.append(object_experiment)
    return pd.concat(pd_list).dropna(axis=1).sort_values(["Object","Distancia (m)"]).reset_index(drop=True)

parser = argparse.ArgumentParser(description='Create an experiment.')
parser.add_argument('-path', required=True)

args = parser.parse_args()
path = pathlib.Path(args.path)

with open(path / "serial.json", "r") as f:
    json_serial = json.load(f)
    serial_path = json_serial['path']
    objects = json_serial['objects']
    distances = json_serial['distances']
    pans = json_serial['pans']
    tilts = json_serial['tilts']

    experiment = create_experiment(objects, distances, pans, tilts)
    experiment.to_csv(path / "experiment.csv")