import os
import json
from sample_configs import sample_config_from_space
from test_configs import test_config
import pickle

num_seed_configs = 5

positive_datasets = []
negative_datasets = []
for i in range(num_seed_configs):
    # Sample a new configuration
    config = sample_config_from_space(None, None)
    print(f"Testing configuration {i+1}/{num_seed_configs}: {config}")
    # Test the configuration
    success, dataset = test_config(config)
    if success:
        print(f"Configuration {i+1} succeeded with dataset.")
        positive_datasets.append((config, dataset))
    elif not success and dataset is not None:
        print(f"Configuration {i+1} failed with dataset.")
        negative_datasets.append((config, dataset))
    else:
        print(f"Configuration {i+1} failed to run the simulation.")

# Save the datasets to files
with open("datasets/positive_datasets.pkl", "wb") as f:
    pickle.dump(positive_datasets, f)

with open("datasets/negative_datasets.pkl", "wb") as f:
    pickle.dump(negative_datasets, f)
