import os
import shutil
import z3
import json
import itertools
import sys

from dtree_checker import DTreeChecker, SeparatorDTreeChecker
from dtree_learner import DTreeLearner
from gurobi_dtree_checker import GurobiDTreeChecker
from oblique_dtree_learner import ObliqueDTreeLearner
from gurobi_dtree_sep_checker import GurobiSeparatorDTreeChecker


class PCSynthesis:
    def __init__(self, dataset_path, time):
        self.dataset_path = dataset_path

        self.base_features = ['x', 'y']
        self.dtree_learner = DTreeLearner(base_features=self.base_features)

        self.npc_min_speed = 2.0
        self.npc_max_speed = 15.0
        self.dtree_checker = SeparatorDTreeChecker(
            time=time
        )

    def mk_temp_dataset(self, dataset_path):
        """
        Create a temporary dataset for learning the decision tree.
        Args:
            dataset (str): Path to the dataset.
        Returns:
            str: Path to the temporary dataset.
        """
        if os.path.exists("out"):
            shutil.rmtree("out")
        os.makedirs("out", exist_ok=True)
        temp_dataset = os.path.join("out", "dataset")
        new_dataset_path = shutil.copyfile(dataset_path, temp_dataset + ".data")
        print(f"Dataset copied to {new_dataset_path}")
        return temp_dataset

    def learn_dtree(self, dataset="dataset/dataset"):
        dataset = self.mk_temp_dataset(dataset)
        self.dtree_learner.write_features(dataset)
        pre = self.dtree_learner.learn(dataset_name=dataset)
        print(f"Learned decision tree: {pre}")
        return pre
    
    def learn_oblique_dtree(self, dataset="classified_dataset.csv"):
        res = self.dtree_learner.learn(dataset)
        print(f"Learned oblique decision tree: {res}")
        return res
    
    def check_oblique_dtree(self, pre):
        cexes = self.dtree_checker.check(pre)
        if len(cexes) > 0:
            print(f"Counterexamples found: {cexes}")
        else:
            print("No counterexamples found, the region is safe.")
        return cexes

    def check_dtree(self, pre, pred_len=2):
        """
        Check the decision tree for safety.
        Args:
            pre (z3.BoolRef): The decision tree to check.
            pred_len (int): Number of predictions to consider.
        Returns:
            bool: True if the region is safe, False otherwise.
        """
        cexs = self.dtree_checker.check(pre, pred_len=pred_len)
        return cexs

    def test(self):
        self.dtree_learner.learn("datasets_pc/dataset_1.5.csv")
        tree = self.dtree_learner.parse_tree_json("dataset/dataset.json")
        cexs = self.dtree_checker.check(tree, pred_len=2)
        if len(cexs) > 0:
            print(f"{len(cexs)} counterexamples found")
        else:
            print("No counterexamples found, the region is safe.")

    def run(self):
        """
        Run the PCSynthesis process.
        """
        max_iters = 1000
        for i in range(max_iters):
            print(f"Iteration {i + 1}/{max_iters}")
            pre = self.learn_dtree(self.dataset_path)
            cexs = self.check_oblique_dtree(pre)
            if len(cexs) > 0:
                for cex in cexs:
                    print(f"Counterexample: {cex}")
                # Update the dataset with counterexamples
                self.dataset_path = self.dtree_learner.add_cexs(self.dataset_path, cexs)
            else:
                break


if __name__ == "__main__":
    if len(sys.argv) < 2:
        dataset_path = "classified_dataset.csv"
    else:
        dataset_path = sys.argv[1]
    pcs = PCSynthesis(dataset_path, 45.5)
    pcs.run()
    # pcs.test()
    print("PCSynthesis completed.")
