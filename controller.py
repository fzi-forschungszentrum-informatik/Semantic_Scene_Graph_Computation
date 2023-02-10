import csv_object_list_dataset_loader as csvloader
import src.computation.coordination as computation_interface
import os
import os.path
import warnings
import time


def check_scene_graph_files(output_dir, num_timestamps):
    number_of_files = len([f for f in os.listdir(output_dir) if f.endswith(
        '.dot') and os.path.isfile(os.path.join(output_dir, f))])
    if number_of_files != num_timestamps:
        warnings.warn(f"There should be exactly as many .dot files as timestamps {number_of_files}/{num_timestamps}")  # noqa


class Controller:
    """Is supposed to act as a Controller as in MVC. Gets initiated and called by the cli module
    or a gui in the future."""

    def main(self, csv_path, osm_path, origin, timestamp_list, clean_load, verbose, outputfolder):
        """Runs the core modules loader and computation one after another.

        Args:
            csv_path (str): The path of the csv file containing the traffic data.
            osm_path (str): The path of the osm file containing the street data.
            origin (tuple(float)): The coordinates of the origin of the used dataset.
            timestamp_list (list(int)): The timestamps to be computed. If an empty list
                is provided, all timestamps of the dataset are computed.
            clean_load (bool): If the Loader module should reload the object structure from
                the csv file if there is an existing pickle.
            verbose (bool, optional): If verbose feedback is wanted. Defaults to False.

        Raises:
            e: a SystemExit which can be raised in the loader module
        """
        loader = csvloader.Loader()
        try:
            loader.load_dataset(
                csv_path, clean_load=clean_load, verbose=verbose)
        except SystemExit as e:
            print("Dataset " + str(csv_path) + ": " + repr(e))
            raise e
        if not isinstance(loader.scenarios[f"-{csv_path}-{0}"], csvloader.Scenario):
            if len(loader.scenarios[f"-{csv_path}-{0}"]) > 1:
                print(f"Multiple Scenarios are found.")
                for key, sub_scenario in loader.scenarios[f"-{csv_path}-{0}"].items():
                    timestamp_list = sub_scenario.timestamps
                    output_dir = f"{outputfolder}/{sub_scenario.id}"
                    coordinator = computation_interface.Coordinator(
                        sub_scenario, osm_path, origin, timestamp_list, verbose,
                        output_dir=output_dir)
                    # computes the semantic scene graphs
                    coordinator.coordinate()
                    check_scene_graph_files(output_dir, len(timestamp_list))

        else:
            scenario = loader.return_scenario(csv_path)
            if not timestamp_list:
                timestamp_list = scenario.timestamps

            output_dir = f"{outputfolder}"
            coordinator = computation_interface.Coordinator(
                scenario, osm_path, origin, timestamp_list, verbose, output_dir=output_dir)
            # computes the semantic scene graphs
            coordinator.coordinate()
            check_scene_graph_files(output_dir, len(timestamp_list))
