import os
import shutil
import z3
import json
import itertools

from dtree_checker import DTreeChecker
from dtree_learner import DTreeLearner


class PCSynthesis:
    def __init__(self, dataset_path):
        self.dataset_path = dataset_path

        state_var = ["time"]
        perception_vars = ["x_0", "y_0", "sin_yaw_0", "cos_yaw_0", "x_1", "y_1", "sin_yaw_1", "cos_yaw_1"]
        self.base_features = state_var + perception_vars
        self.dtree_learner = DTreeLearner(base_features=self.base_features)

        self.npc_min_speed = 3.0
        self.npc_max_speed = 15.0
        self.dtree_checker = DTreeChecker(npc_speeds=[self.npc_min_speed, self.npc_max_speed])

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
        pre = self.dtree_learner.learn(dataset=dataset)
        print(f"Learned decision tree: {pre}")
        return pre
    
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
        tree = self.dtree_learner.get_pre_from_json("out/dataset.json")
        cexs = self.dtree_checker.check(tree, pred_len=2)
        if len(cexs) > 0:
            print(f"Counterexamples found: {cexs}")
        else:
            print("No counterexamples found, the region is safe.")
        _ = self.dtree_learner.add_cexs("out/dataset.data", cexs[0])

    def run(self):
        """
        Run the PCSynthesis process.
        """
        max_iters = 10
        for i in range(max_iters):
            print(f"Iteration {i + 1}/{max_iters}")
            pre = self.learn_dtree(self.dataset_path)
            cexs = self.check_dtree(pre)
            if len(cexs) > 0:
                print(f"Counterexamples found: {cexs}")
                for cex in cexs:
                    print(f"Counterexample: {cex}")
                # Update the dataset with counterexamples
                self.dataset_path = self.dtree_learner.add_cexs(self.dataset_path, cexs)
            else:
                print("No counterexamples found, the region is safe.")
                break


if __name__ == "__main__":
    dataset_path = "dataset/new_dataset.data"
    pcs = PCSynthesis(dataset_path)
    pcs.run()
    print("PCSynthesis completed.")
            

    