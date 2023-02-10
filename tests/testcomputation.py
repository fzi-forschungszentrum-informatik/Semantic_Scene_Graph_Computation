import unittest
import src.computation as computation
import csv_object_list_dataset_loader as csvloader
import lanelet2
import os
import igraph

file_dir = str(os.path.dirname(os.path.realpath(__file__)))


def create_roadgraph_graph(lanelet_map):
    """Creates the igraph graph for the roadgraph.

    Args:
        lanelet_map (lanelet2.core.LaneletMap): The lanelet map created by lanelet2

    Returns:
        igraph.Graph: the roadgraph in the igraph format
    """
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle
    )
    routing_graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules)
    file_path = f"./{hash(lanelet_map)}transformed_graph.graphml"
    routing_graph.exportGraphML(file_path)
    graph = igraph.Graph().Read_GraphML(file_path)
    os.remove(file_path)
    return graph


class TestTaf(unittest.TestCase):
    """Test case for the taf dataset"""

    def setUp(self):
        osm_path = file_dir + "/data/K733.osm"
        origin = (
            49.005306,
            8.4374089,
        )

        csv_path = file_dir + "/data/vehicle_tracks_000.csv"
        self.pdata_path = file_dir + "/data/vehicle_tracks_000.pdata"
        loader = csvloader.Loader()
        loader.load_dataset(csv_path)
        scenario = loader.return_scenario(csv_path)
        timestamp = 400
        scene = scenario.get_scene(timestamp)

        lanelet_map = computation.read_to_lanelet_map(osm_path, origin)

        matching_dict = computation.ProbabilisticMatchingDict(
            scene, lanelet_map)
        roadgraph = computation.Roadgraph(
            lanelet_map, matching_dict, create_roadgraph_graph(lanelet_map)
        )
        projection_identity_dict = computation.ProjectionIdentityDict(
            matching_dict)

        self.semantic_scene_graph = computation.SemanticSceneGraph(
            matching_dict, scene, roadgraph, projection_identity_dict, verbose=True,
            # SHORTEST, MOST_LIKELY, KEEP_ALL, NO_PARALLEL
            edge_mode=computation.EdgeMode.NO_PARALLEL
        )

    def test_create_dot_file(self):
        self.semantic_scene_graph.write_dot("./semantic_scene_graph.dot")

    def tearDown(self):
        if os.path.exists(self.pdata_path):
            os.remove(self.pdata_path)


class TestShowData(unittest.TestCase):
    """Not really a test. Visualizes data from computation"""

    def setUp(self):
        # using TAF dataset
        osm_path = file_dir + "/data/K733.osm"
        origin = (
            49.005306,
            8.4374089,
        )
        csv_path = file_dir + "/data/vehicle_tracks_000.csv"
        self.pdata_path = file_dir + "/data/vehicle_tracks_000.pdata"
        loader = csvloader.Loader()
        loader.load_dataset(csv_path)
        lanelet_map = computation.read_to_lanelet_map(osm_path, origin)
        self.timestamp = 400
        self.scenario = loader.return_scenario(csv_path)
        self.entity_id_list = self.scenario.entity_ids
        self.lanelet_list = []
        for lanelet in lanelet_map.laneletLayer:
            self.lanelet_list.append(lanelet)

    def test_print_lanelets(self):
        """Function to print out a list of lanelets from the lanelet_map

        Args:
            lanelet_list (list): list of lanelets to be printed
        """
        for lanelet in self.lanelet_list:
            print(lanelet)

    def tearDown(self):
        if os.path.exists(self.pdata_path):
            os.remove(self.pdata_path)


if __name__ == "__main__":
    unittest.main()
