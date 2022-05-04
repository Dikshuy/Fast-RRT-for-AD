# Fast RRT for AD

We have tried to implement Fast RRT algorithm for autonomous driving task. 

You need to install the following python libraries before running the codes:
```bash
- shapely
- descartes
- networkx
- bisect
```

The `environment.py` creates an environment where all the task are being deployed using the `obs.yaml` or `road.yaml` files. The environment can be made more complex by adding obstacles in these yaml files. 

The `algo.py` contains three RRT variants:

1. RRT
2. RRT*
3. Fast RRT

In the fast RRT, we use rule templates and then deploy aggresive strategy to reach to our goal location. The complete implementation was summarised during the presentation. 

The `main.py` will use all the above files and generate the required results.
