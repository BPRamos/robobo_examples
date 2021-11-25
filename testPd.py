from doepy import build
import pandas as pd

objects = [ ("apple", "./../../robobo-gazebo-models/apple/model.sdf"),
                    ("dog", "./../../robobo-gazebo-models/dog/model.sdf"),
                    ]
distances = [0, 0.05, 0.10]
tilts = [None]
pans = [None]

#df['new'] = 0

pd_list = []
for object in objects:
    object_experiment = build.full_fact(
        {'Distancia (m)':   distances,
        'Pan (deg)':        pans,
        'Tilt (deg)':       tilts,}
    )
    object_experiment["Object"] = object[0]
    object_experiment["Object_path"] = object[1]
    pd_list.append(object_experiment)

experiment = pd.concat(pd_list).dropna(axis=1).sort_values(["Object","Distancia (m)"]).reset_index(drop=True)
print(experiment["Object"])