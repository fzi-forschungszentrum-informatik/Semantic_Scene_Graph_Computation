import time
import ray
import lanelet2
import igraph
import os
import src.computation as computation

# fewer than 20 ts will be computed in serial manner not parallel
MAX_SERIAL_TIMESTAMPS = 20


class Coordinator:
    def __init__(
        self, scenario, osm_path, origin, timestamp_list, verbose, output_dir="./dotgraphs"
    ):
        self.scenario = scenario
        self.osm_path = osm_path
        self.origin = origin
        self.verbose = verbose
        self.timestamp_list = timestamp_list
        self.output_dir = output_dir
        self.results = []

        # find cropping box
        self._max_x, self._min_x = -10e9, 10e9
        self._max_y, self._min_y = -10e9, 10e9
        margin = 100
        for entity_id, entity in self.scenario._entity_dict.items():
            for entity_state in entity._entity_states_dict.values():
                self._max_x = max(entity_state.x, self._max_x)
                self._max_y = max(entity_state.y, self._max_y)
                self._min_x = min(entity_state.x, self._min_x)
                self._min_y = min(entity_state.y, self._min_y)
        self._max_x += margin
        self._max_y += margin
        self._min_y -= margin
        self._min_y -= margin

    def coordinate(self):

        lanelet_map = computation.read_to_lanelet_map(
            self.osm_path, self.origin, (self._max_x, self._min_x, self._max_y, self._min_y))

        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle
        )
        routing_graph = lanelet2.routing.RoutingGraph(
            lanelet_map, traffic_rules)
        file_path = f"./{hash(lanelet_map)}transformed_graph.graphml"

        routing_graph.exportGraphML(file_path)
        graph = igraph.Graph().Read_GraphML(file_path)
        os.remove(file_path)

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        if len(self.timestamp_list) < MAX_SERIAL_TIMESTAMPS:
            for timestamp in self.timestamp_list:
                self.compute(timestamp, graph)
        else:
            # Start Ray
            tic = time.perf_counter()
            if not ray.is_initialized():
                ray.init(ignore_reinit_error=True)
            toc = time.perf_counter()
            if self.verbose:
                print(f"STARTING RAY TOOK {toc-tic:0.4f} seconds")
            for timestamp in self.timestamp_list:
                results = self.compute_in_parallel.remote(
                    self, timestamp, graph)
            ray.get(results)

    @ray.remote
    def compute_in_parallel(self, timestamp, graph):
        """Runs the compute function in parallel

        Args:
            timestamp (int): the timestamp to compute
            graph (igraph.Graph): the routing graph of the scenario
        """
        self.compute(timestamp, graph)

    def compute(self, timestamp, graph):
        """Takes all the necessary steps to compute a Semantic Scene Graph. Writes the finished
        graph to the specified output directory

        Args:
            timestamp (int): the timestamp to compute
            graph (igraph.Graph): the routing graph of the scenario
        """
        scene = self.scenario.get_scene(timestamp)
        lanelet_map = computation.read_to_lanelet_map(
            self.osm_path, self.origin, (self._max_x, self._min_x, self._max_y, self._min_y))
        matching_dict = computation.ProbabilisticMatchingDict(
            scene, lanelet_map)
        roadgraph = computation.Roadgraph(
            lanelet_map, matching_dict, graph, self.verbose)
        projection_identity_dict = computation.ProjectionIdentityDict(
            matching_dict, self.verbose)

        semantic_scene_graph = computation.SemanticSceneGraph(
            matching_dict, scene, roadgraph, projection_identity_dict,
            # SHORTEST, MOST_LIKELY, KEEP_ALL, NO_PARALLEL
            edge_mode=computation.EdgeMode.MOST_LIKELY
        )
        semantic_scene_graph.write_dot(
            f"{self.output_dir}/semantic-scene-graph_{timestamp}.dot")
