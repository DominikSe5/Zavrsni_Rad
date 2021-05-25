import yaml
from yaml import loader

data = yaml.load('data.yaml', Loader=yaml.FullLoader)
print(data)
