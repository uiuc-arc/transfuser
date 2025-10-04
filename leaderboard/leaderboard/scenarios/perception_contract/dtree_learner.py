import z3
import json
import itertools
import subprocess

# from obliquetree.utils import export_tree
# from obliquetree import Classifier
import pandas as pd
import numpy as np


class DTreeLearner:
    def __init__(self, base_features=[], path_to_c5="./c50exact/c5.0dbg"):
        self.cmd = f"{path_to_c5} -I 1 -m 1 -f "
        self.base_features = base_features
        self.derived_feature_map = {}
        self._var_coeff_map = {}

    def _generate_derived_features(self, k=2):
        res = []
        for var in self.base_features:
            var_coeff_map = {var: -1}
            expr = f"(-1*{var})"
            name = expr
            res.append((name, (var_coeff_map, expr)))

        if len(self.base_features) < k:
            return res

        coeff_combinations = list(itertools.product([-5, -4, -3, -2, -1, 1, 2, 3, 4, 5], repeat=k))
        var_id_iter = range(len(self.base_features))
        for selected_var_ids in itertools.combinations(var_id_iter, k):
            for coeff in coeff_combinations:
                var_coeff_map = {
                    self.base_features[i]: c for c, i in zip(coeff, selected_var_ids)
                }
                expr = " + ".join(
                    f"({c}*{self.base_features[i]})"
                    for c, i in zip(coeff, selected_var_ids)
                )
                name = f"({expr})"
                res.append((name, (var_coeff_map, expr)))
        return res

    def generate_features(self):
        for k in range(2, len(self.base_features) + 1):
            self.derived_feature_map.update(self._generate_derived_features(k))

        self._var_coeff_map.update([(var, {var: 1}) for var in self.base_features])
        self._var_coeff_map.update(
            [
                (var, coeff_map)
                for var, (coeff_map, _) in self.derived_feature_map.items()
            ]
        )

    def write_features(self, path):
        file_lines = (
            ["precondition."]
            + [f"{var}:  continuous." for var in self.base_features]
            + [
                f"{var} := {expr}."
                for var, (_, expr) in self.derived_feature_map.items()
            ]
            + ["precondition:  true, false."]
        )
        with open(path + ".names", "w") as f:
            f.write("\n".join(file_lines))

    def learn(self, dataset_name="dataset/dataset"):
        self.generate_features()
        self.write_features(dataset_name)

        # ObliqueTree Classifier
        # dataset = pd.read_csv(dataset_name + ".data", header=None)
        # X = dataset.iloc[:, :-1]
        # y = dataset.iloc[:, -1]
        # clf = Classifier(
        #     use_oblique=True,
        #     max_depth=-1,
        #     random_state=42,
        #     n_pair=X.shape[1],
        #     categories=[]
        # )
        # clf.fit(X, y)
        # print(f"Writing to {dataset_name}.json")
        # export_tree(clf, f"{dataset_name}.json")
        # ObliqueTree Classifier

        proc = subprocess.Popen(self.cmd + dataset_name, shell=True)
        output, error = proc.communicate()
        if proc.returncode != 0:
            raise RuntimeError(f"Error running C5.0: {error}")

        return self.get_pre_from_json(f"{dataset_name}.json")

    def get_pre_from_json(self, path):
        try:
            with open(path) as json_file:
                tree = json.load(json_file)
                return self.parse_tree(tree)
        except json.JSONDecodeError:
            raise ValueError(f"Cannot parse {path} as a JSON file")

    def parse_tree(self, tree) -> z3.BoolRef:
        if tree["children"] is None:
            # At a leaf node, return the clause
            if tree["classification"]:
                return z3.BoolVal(True)  # True leaf node
            else:
                return z3.BoolVal(False)  # False leaf node
        elif len(tree["children"]) == 2:
            # Post-order traversal
            left = self.parse_tree(tree["children"][0])
            right = self.parse_tree(tree["children"][1])
            # Create an ITE expression tree
            z3_expr = z3.Sum(
                *(
                    coeff * z3.Real(base_fvar)
                    for base_fvar, coeff in self._var_coeff_map[
                        tree["attribute"]
                    ].items()
                )
            )
            z3_cut = z3.simplify(z3.fpToReal(z3.FPVal(tree["cut"], z3.Float64())))
            if z3.is_true(left):
                if z3.is_true(right):
                    return z3.BoolVal(True)
                elif z3.is_false(right):
                    return z3_expr <= z3_cut
            if z3.is_false(left):
                if z3.is_true(right):
                    return z3_expr > z3_cut
                elif z3.is_false(right):
                    return z3.BoolVal(False)
            # else:
            return z3.If((z3_expr <= z3_cut), left, right)
        else:
            raise ValueError("error parsing the json object as a binary decision tree)")

    def parse_oblique_tree(self, tree):
        """
        Parse the oblique tree structure into a Z3 expression.
        Args:
            tree (dict): The oblique tree structure.
        Returns:
            z3.BoolRef: The Z3 expression representing the decision tree.
        """
        raise NotImplementedError

    def add_cexs(self, dataset_path, cexs):
        """
        Update the dataset with counterexamples.
        Args:
            dataset_path (str): Path to the dataset.
            cexs (list): List of counterexamples to add.
        Returns:
            str: Path to the updated dataset.
        """
        existing_points = set()
        with open(dataset_path, "r") as f:
            lines = f.readlines()
            existing_points = set(line.strip() for line in lines)
        with open(dataset_path, "a") as f:
            for cex in cexs:
                string = ",".join(map(str, cex)) + ",false\n"
                if string.strip() not in existing_points:
                    print(f"Adding counterexample: {string.strip()}")
                    f.write(string)
                else:
                    print(f"Counterexample already exists: {string.strip()}")
        return dataset_path


if __name__ == "__main__":
    input = ["time"]
    prediction = [
        "x_0",
        "y_0",
        "sin_yaw_0",
        "cos_yaw_0",
        "x_1",
        "y_1",
        "sin_yaw_1",
        "cos_yaw_1",
    ]
    learner = DTreeLearner(
        base_features=input + prediction,
    )
    tree = learner.get_pre_from_json("out_v1/dtree.json")
    print(tree)
