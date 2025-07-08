import yaml

with open("base.yaml", "r") as f:
    data = yaml.safe_load(f)
    print(data)
