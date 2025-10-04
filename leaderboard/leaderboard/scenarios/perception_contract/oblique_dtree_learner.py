import z3
import json
import itertools
import subprocess

from obliquetree.utils import export_tree, visualize_tree
from obliquetree import Classifier
import sys
import os

import pandas as pd
import numpy as np


class ObliqueDTreeLearner:
    def __init__(self, base_features=['x', 'y']):
        self.base_features = base_features
        self.truth_map = {}
        self.feature_map = {}

    def _prepare_features(self, labels):
        for i, feature in enumerate(self.base_features):
            self.feature_map[i] = z3.Real(feature)
        
        if all(labels):
            self.truth_map[0.0] = True
            labels = [0.0 if label else 0.0 for label in labels]
        else:
            self.truth_map[1.0] = True
            self.truth_map[0.0] = False

        return labels

    def learn(self, dataset_path, visualize=False):
        """
        Learn an oblique decision tree from the dataset.
        Args:
            dataset_path (str): Path to the dataset.
        Returns:
            z3.BoolRef: The learned decision tree as a Z3 expression.
        """
        dataset = pd.read_csv(dataset_path, header=None)
        X = dataset.drop(columns=[2])
        y = dataset[2]
        y = self._prepare_features(y)
        print(f"New labels: {y}")
        clf = Classifier(
            use_oblique=True,
            max_depth=-1,
            random_state=42,
            n_pair=X.shape[1],
            categories=[]
        )
        clf.fit(X, y)
        tree_path = os.path.join(os.path.dirname(dataset_path), "tree.json")
        export_tree(clf, tree_path)
        tree_json = None
        with open(tree_path, 'r') as file:
            tree_json = json.load(file)
            tree_json = tree_json['tree']
        tree_image_path = os.path.join(os.path.dirname(dataset_path), "tree.png")
        if visualize:
            visualize_tree(clf, save_path=tree_image_path)
        return self.parse_tree_json(tree_json)

    def parse_tree_json(self, tree):
        """
        Parse the decision tree JSON structure into a Z3 expression.
        Args:
            tree (dict): The decision tree in JSON format.
        Returns:
            z3.BoolRef: The Z3 expression representing the decision tree.
        """
        if tree["is_leaf"]:
            return z3.BoolVal(self.truth_map[tree["value"]])
        
        left = self.parse_tree_json(tree["left"])
        right = self.parse_tree_json(tree["right"])

        z3_expr = None
        if tree["is_oblique"]:
            z3_expr = z3.Sum(
                *(
                    w * self.feature_map[i] for w, i in zip(tree["weights"], tree['features'])
                )
            )
        else:
            z3_expr = self.feature_map[tree['feature_idx']]

        z3_threshold = z3.simplify(z3.fpToReal(z3.FPVal(tree["threshold"], z3.Float64())))

        if z3.is_true(left):
            if z3.is_true(right):
                return z3.BoolVal(True)
            elif z3.is_false(right):
                return z3_expr <= z3_threshold
        if z3.is_false(left):
            if z3.is_true(right):
                return z3_expr > z3_threshold
            elif z3.is_false(right):
                return z3.BoolVal(False)

        return z3.If((z3_expr <= z3_threshold), left, right)
    
    def add_cexs(self, dataset_path, cexs):
        """
        Add counterexamples to the dataset.
        Args:
            dataset_path (str): Path to the dataset.
            cexs (list): List of counterexamples to add.
        Returns:
            str: Path to the updated dataset.
        """
        dataset = pd.read_csv(dataset_path, header=None)
        if 1.0 not in self.truth_map:
            self.truth_map[1.0] = True
            self.truth_map[0.0] = False
        for cex in cexs:
            for point in cex:
                if type(point[0]) is not float:
                    new_row = [point[0].item(), point[1].item(), 0.0]
                else:
                    new_row = [point[0], point[1], 0.0]  
            dataset = pd.concat([dataset, pd.DataFrame([new_row])], ignore_index=True)
        dataset.to_csv(dataset_path, index=False)
        print(f"Counterexamples added to {dataset_path}")
        return dataset_path
            
