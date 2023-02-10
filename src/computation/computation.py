# use `export PYTHONPATH=../../` in the same directory as computation.py so this import works
from __future__ import annotations
from enum import Enum
import csv_object_list_dataset_loader as csvloader
import igraph
import lanelet2
from collections import defaultdict


DECIMAL_PLACES = 3


class ProbabilisticMatchingDict(dict):
    """Dict containing entity_ids and their list of matches.

    The dictionary uses entity_id as keys with the corresponding value being a list of matches.
    Example:
    >>> {some_entity_id: [some_match1, some_match2]}
    """

    def __init__(
        self,
        scene: csvloader.Scene,
        lanelet_map: lanelet2.core.LaneletMap,
        von_mises_kappa=2.0,
        max_dist=1.0,
    ):
        """Creates a probabilistic matching dictionary by matching Entities to lanelets of
        the traffic scene.
        Each entity is matched to a lanelet based on its current position and yaw on a lanelet,
        the covariance of its position and the kappa of the
        von Mises distribution.
        The matches have to be in range of a maximum euler distance in order to be considered as
        a match. The probabilistic matches are later used to create projection identities, which
        help to determine relations between the distinct entities.

        Args:
            scenario(csvloader.Scenario): Scene object depicting every Entity for one timestamp
            (every EntityState matching one timestamp).
            timestamp (int): time which the traffic participants are to be matched to the lanelets
            von_mises_kappa (int, optional): Kappa of the von Mises distribution. Defaults to 2.
            max_dist (int, optional): Euclidean distance threshold for matching. Defaults to 1.
        """
        entity_states = scene.entity_states
        for entity_state in entity_states:
            # Create ObjectwithCovariance2d for each entity.
            if isinstance(entity_state.entity_id, str):
                tmp_entity_id = 0  # this id is not needed for later computation
            else:
                tmp_entity_id = entity_state.entity_id
            current_obj_with_cov = lanelet2.matching.ObjectWithCovariance2d(
                tmp_entity_id,
                lanelet2.matching.Pose2d(
                    entity_state.x, entity_state.y, entity_state.yaw),
                [],
                lanelet2.matching.PositionCovariance2d(
                    entity_state.x, entity_state.y, entity_state.yaw
                ),
                von_mises_kappa,
            )
            # Get all probabilistic lanelet matches of the ObjectWithCovariance2d with a maximum
            # deterministic euler distance of max_dist, in ascending order by Mahalanobis distance.
            match_list = lanelet2.matching.getProbabilisticMatches(
                lanelet_map, current_obj_with_cov, max_dist
            )
            if match_list:
                # Save matches in a dictionary with the entity_id as key and always take
                # the first match, as match_list is sorted in ascending order by the squared
                # Mahalanobis Distance so the distance of the other matches can never be
                # smaller than the first one.
                seen = set()
                self[entity_state.entity_id] = [
                    match
                    for match in match_list
                    if match.lanelet.id not in seen and not seen.add(match.lanelet.id)
                ]


class ProjectionIdentityDict(dict):
    """Dict-of-a-dict structure containing entity_ids, lanelet ids and probability of matching.

    The outermost dictionary uses entity_id as keys with the corresponding value being a
    dictionary with lanelet ids as keys and the probability of the match as value.
    This looks like this:
    >>> {entity_id: {lanelet_id: matching_probability}}
    Example:
    >>> {entity_id: {lanelet_id: matching_probability0, other_lanelet_id: matching_probability1}}
    """

    def __init__(
        self,
        matching_dict: dict[int, list[lanelet2.matching.ConstLaneletMatchProbabilistic]],
        verbose=False,
    ):
        """Calculates the probability of every single match in the matching_dict.

        This is achieved by dividing the given mahalanobis distance of each match through the sum
        of all mahalanobis distances of the matches corresponding to the entity.

        Args:
            matching_dict (dict[int, list[lanelet2.matching.ConstLaneletMatchProbabilistic]]):
                Should contain entity_id as keys and a list of all possible matches as their value.
            verbose (bool, optional): Set to True for additional output. Defaults to False.
        """
        # took the absolute value of the mahalanobisDistSq, so the probability should make more
        # sense now. Before values above 1 were possible.
        self.__keys_swapped = {}
        for entity_id in matching_dict:
            dist_sum = 0
            for match in matching_dict[entity_id]:
                dist_sum += abs(match.mahalanobisDistSq)

            self[entity_id] = {}

            for match in matching_dict[entity_id]:
                probability = abs(match.mahalanobisDistSq) / dist_sum
                lanelet = match.lanelet

                self[entity_id][lanelet.id] = round(probability, 6)
                if lanelet.id not in self.__keys_swapped:
                    self.__keys_swapped[lanelet.id] = {}
                self.__keys_swapped[lanelet.id][entity_id] = probability
                if verbose:
                    print(lanelet, entity_id, probability,)

    @property
    def swap_keys(self) -> dict[int, dict[int, float]]:
        """Same dict-of-a-dict structure as ProjectionIdentityDict but with entity id
        and lanelet id swapped.

        Returns:
            dict[int, dict[int, float]]: First key is entity id, second key is lanelet id
                and the contained value is the probability.
        """
        return self.__keys_swapped


class Roadgraph:
    """Graph structure representing a road section taking into account the local traffic rules."""

    def __init__(
        self,
        lanelet_map: lanelet2.core.LaneletMap,
        matching_dict: ProbabilisticMatchingDict,
        graph,
        verbose=False,
    ):
        """Create Roadgraph by first importing a premade set of traffic_rules amd then creating
        the RoutingGraph using the lanelet_map and the traffic_rules.

        The graph is converted to the igraph format by using a workaround. The routing_graph is
        first exported in GraphML format and then imported in igraph format.
        This is done because igraph is currently not supporting reading the .dot format.

        Lastly all shortest paths between lanelets containing a match are calculated.
        A path is defined by a list of continuous lanelets in the order in which they are
        traversed to reach the destination.

        Args:
            lanelet_map (lanelet2.core.LaneletMap): map generated by lanelet2 from an osm file
                and the coordinates of the road segment represented in the map
            matching_dict (ProbabilisticMatchingDict): dict containing entity_ids and
                their list of matches
        """
        self._graph = graph

        self._lanelet_dict = {
            lanelet.id: lanelet for lanelet in lanelet_map.laneletLayer}

        self._matching_dict = matching_dict
        self._compute_shortest_relation_paths(
            lanelet_map, matching_dict, verbose)

    def _compute_shortest_relation_paths(self, lanelet_map, matching_dict, verbose):
        # All lanelet ids with a match.
        matched_lanelet_ids = set(
            match.lanelet.id for matching_list in matching_dict.values() for match in matching_list
        )

        # Dict for conversion from lanelet ID to igraph ID
        lanelet_to_igraph_id = {
            lanelet.id: index for index, lanelet in enumerate(lanelet_map.laneletLayer)
        }

        matched_lanelet_igraph_ids = [
            lanelet_to_igraph_id[lanelet_id] for lanelet_id in matched_lanelet_ids
        ]

        overlapping_igraph_ids = {
            edge.tuple[1] for edge in self._graph.es if edge["relation"] == "Conflicting"
        }

        # Delete conflicting edges from the graph so conflicting edges will not be traversed and
        # used as a part of a shortest path.
        conflicting_edges = self._graph.es.select(relation_eq="Conflicting")
        graph_without_conflicts = self._graph.copy()
        graph_without_conflicts.delete_edges(conflicting_edges)

        # The matched Lanelets + Lanelets that are part of an conflicting / overlapping relation
        targeted_igraph_ids = list(
            {*matched_lanelet_igraph_ids, *overlapping_igraph_ids})

        """
        Calculate all shortest path from a given node to every node in a graph and put it in
        a dictionary with lanelet id as keys and another inner dictionary as value.
        We are searching in a roadgraph without conflicts because we want to avoid conflicting
        relations in the middle of paths as they can result in non-valid paths. See dummy Lanelets
        and __merge_paths function for more information.
        The inner dictionary holds the path represented by a list of roadgraph edge relations
        with the target lanelet being the key.
        Example:
        >>> {from_lanelet_id: {to_lanelet0_id: [path as a list], to_lanelet1_id: [path as a list]}}
        The paths are defined by edge tuples such as
        >>> (from_lanelet_id, "relation", weight: float, to_lanelet_id).
        If there is a path 1->2->3->4 and all relations were successors, the dict entry would be:
        {1: {4: [(1, "Successor", <weight>, 2), (2, "Successor", <weight>, 3), (3, "Successor", <weight>, 4)]}} 
        # noqa
        """
        self.__shortest_relation_path_dict = {
            lanelet_id: {
                self._lanelet_id(targeted_igraph_ids[index]): [
                    (
                        self._lanelet_id(
                            graph_without_conflicts.es[edge_id].tuple[0]),
                        graph_without_conflicts.es[edge_id]["relation"],
                        graph_without_conflicts.es[edge_id]["routingCost"],
                        self._lanelet_id(
                            graph_without_conflicts.es[edge_id].tuple[1]),
                    )
                    for edge_id in edge_id_relation_path_list
                ]
                for index, edge_id_relation_path_list in enumerate(
                    graph_without_conflicts.get_shortest_paths(
                        lanelet_to_igraph_id[lanelet_id],
                        to=targeted_igraph_ids,
                        weights=graph_without_conflicts.es["routingCost"],
                        output="epath",
                    )
                )
            }
            for lanelet_id in matched_lanelet_ids
        }

        # List of tuples of lanelet ids (int, int) that describe all conflicting relations.
        # These have been deleted from the graph_without conflicts
        conflicting_relations = [
            (self._lanelet_id(edge.tuple[0]), self._lanelet_id(edge.tuple[1]))
            for edge in conflicting_edges
        ]

        # Re-append conflicting relations to the end of the paths in shortest_relation_path_dict if
        # the last lanelet of the path is the same as the first lanelet of the conflicting relation
        # This has to be done because we removed all conflicting relations before searching for
        # shortest paths.
        # TODO: optimize runtime-wise:
        #   Paths only consisting of a conflicting relation are currently not respected.
        for edge_tuple in conflicting_relations:
            for (
                from_lanelet,
                to_lanelet_and_path_list,
            ) in self.__shortest_relation_path_dict.items():
                for path in to_lanelet_and_path_list.values():
                    path_found = False
                    if path and path[-1][-1] == edge_tuple[0] and path[-1][1] != "Conflicting":
                        new_path = path + [
                            (
                                edge_tuple[0],
                                "Conflicting",
                                1.0,
                                edge_tuple[1],
                            )
                        ]
                        path_found = True
                    elif not path and from_lanelet == edge_tuple[0]:
                        # The two lanelets just have a direct overlapping/conflicting relation
                        new_path = [
                            (edge_tuple[0], "Conflicting", 1.0, edge_tuple[1])]
                        path_found = True

                    if path_found and (
                        (not to_lanelet_and_path_list[edge_tuple[1]])
                        or self.__first_path_longer(
                            to_lanelet_and_path_list[edge_tuple[1]], new_path
                        )
                    ):
                        to_lanelet_and_path_list[edge_tuple[1]] = new_path

        # List of all merged paths that still have to be added to the shortest_relation_path_dict
        merged_paths = []
        # All paths to one specific igraph id (the equivalent lanelet id) are selected and merged
        # by self.__merge_paths()
        for overlapping_igraph_id in overlapping_igraph_ids:
            target_lanelet = self._lanelet_id(overlapping_igraph_id)
            paths_to_merge = [
                (
                    origin,
                    self.__shortest_relation_path_dict[origin][target_lanelet],
                )
                for origin in self.__shortest_relation_path_dict.keys()
                if self.__shortest_relation_path_dict[origin][target_lanelet]
            ]
            merged_paths.extend(self.__merge_paths(paths_to_merge))

        # The merged paths are added to the shortest_relation_path_dict if they are shorter than
        # the existing paths.
        # This way we also choose the shortest path between two lanelets from all the merged paths
        # It has to be done outside of __merge_paths so that the already merged paths are not
        # selected as a path to merge.
        # Doing this inside of __merge_paths would also result in missing out on some
        # shortest paths.
        for intersecting_path in merged_paths:
            target_lanelet_id = intersecting_path[-1][-1]
            origin_lanelet_id = intersecting_path[0][0]

            if not (
                target_lanelet_id in self.__shortest_relation_path_dict[origin_lanelet_id]
                and self.__shortest_relation_path_dict[origin_lanelet_id][target_lanelet_id]
                and self.__first_path_longer(
                    intersecting_path,
                    self.__shortest_relation_path_dict[origin_lanelet_id][target_lanelet_id],
                )
            ):
                self.__shortest_relation_path_dict[origin_lanelet_id][
                    target_lanelet_id
                ] = intersecting_path

        # deletes all the remaining empty dummy entries in shortest_relation_path_dict
        for shortest_path_dict in self.__shortest_relation_path_dict.values():
            for overlapping_igraph_id in overlapping_igraph_ids:
                # We only want to delete paths to our dummies here not
                # paths to actual entities which also happen to be the
                # target of an overlapping relation.
                if self._lanelet_id(overlapping_igraph_id) not in matched_lanelet_ids:
                    dummy_lanelet = self._lanelet_id(overlapping_igraph_id)
                    if dummy_lanelet in shortest_path_dict.keys():
                        shortest_path_dict.pop(dummy_lanelet)

        if verbose:
            print("Roadgraph finshed")

    def __merge_paths(self, path_list) -> list[list[tuple[int, str, float, int]]]:
        """Merges the provided paths and adds them to the shortest_relation_path_dict if they are
        shorter than the previously calculated ones.
        All provided paths have to lead to the same Lanelet. These paths are then split up into
        paths that end with a "Conflicting" relation and those that don't.
        In the next step they are merged and inserted into the shortest_relation_path_dict after
        assuring that they are actually shorter. Finally all provided unmerged paths are deleted
        from shortest_relation_path_dict if the Lanelet they lead to is a dummy Lanelet.
        A dummy lanelet is a lanelet which has an intersecting relation with another lanelet and
        is used to create a path between two lanelets A & B. This path is created by merging the
        path from lanelet A to the dummy lanelet and the path from lanelet B to the dummy lanelet.



        Args:
            path_list (list[int, list[tuple(int, str, float, int)]]): The List of paths to be
            merged, all leading to the same Lanelet.

        Returns:
            calculated_paths (list[list[tuple(int, str, float, int)]]): The List of the merged
            paths. These still have to be added to the shortest_relation_path_dict
        """
        # paths with overlapping relation
        overlapping_paths = [
            # path[1] checks if the path even exists, maybe should be checked earlier
            path
            for path in path_list
            if path[-1][-1][1] == "Conflicting"
        ]
        non_overlapping_paths = [
            path for path in path_list if path[1] and path not in overlapping_paths
        ]

        calculated_paths = []

        for overlapping_path in overlapping_paths:

            for non_overlapping_path in non_overlapping_paths:
                # the path from the target lanelet to the dummy lanelet is reverted to get a
                # directed path from
                # the origin lanelet to the dummylanelet to the target lanelet
                # (from_lanelet, relation, routing_cost, to_lanelet) becomes (to_lanelet,
                # relation, routing_cost, from_lanelet)
                temp_non_overlapping_path = [
                    (sub[3], sub[1], sub[2], sub[0]) for sub in non_overlapping_path[1]
                ]
                temp_non_overlapping_path.reverse()
                intersecting_path = overlapping_path[1] + \
                    temp_non_overlapping_path
                # Add merged path to the calculated_paths list that is later returned
                calculated_paths.append(intersecting_path)

        return calculated_paths

    def __first_path_longer(self, first_path, second_path) -> bool:
        """Checks if the first provided path is longer than the second one.

        Args:
            first_path (list[int, list[tuple(int, str, float, int)]]): first path to be measured
            second_path (list[int, list[tuple(int, str, float, int)]]): second path to be measured

        Returns:
            bool: If the first path is longer than the second one.
        """
        return self.__calculate_path_length(first_path) > self.__calculate_path_length(second_path)

    def __calculate_path_length(self, path) -> float:
        """Calculates the length of a path based on the edge weight
         (routingCost provided by Lanelet2)

        Args:
            path (list[int, list[tuple(int, str, float, int)]]): The path to be measured.

        Returns:
            float: The total length of the path based on the lanelet2 routingCost
        """
        return sum([step[2] for step in path])

    def _lanelet_id(self, vertex_id: int) -> int:
        """Gets the equivalent Lanelet id to the provided igraph vertex id.

        Args:
            vertex_id (int): The igraph id to be converted into the lanelet format

        Returns:
            int: The lanelet id of the Lanelet
        """
        return int(self._graph.vs[vertex_id]["info"].split("[id: ", 1)[1].split(",", 1)[0])

    def get_shortest_relation_path(
        self, from_lanelet: lanelet2.core.ConstLanelet, to_lanelet: lanelet2.core.ConstLanelet
    ) -> list[int]:
        """Shortest relation path from a lanelet of the roadgraph to another lanelet in
        the roadgraph.

        Args:
            from_lanelet (lanelet2.core.ConstLanelet): Origin lanelet.
            to_lanelet (lanelet2.core.ConstLanelet): Target lanelet.

        Returns:
            list[int]: Shortest path represented by a list of roadgraph edge relations.
        """
        return self.__shortest_relation_path_dict[from_lanelet.id][to_lanelet.id]

    def get_all_shortest_relation_paths(
        self, from_lanelet: lanelet2.core.ConstLanelet
    ) -> dict[int, list[int]]:
        """Shortest relation paths from a lanelet of the roadgraph to all possible lanelets in
        the roadgraph.

        Args:
             from_lanelet (lanelet2.core.ConstLanelet): Origin lanelet.

         Returns:
             dict[int, list[int]]: Dictionary of shortest paths represented by a list roadgraph
                edge relations.
        """
        return self.__shortest_relation_path_dict[from_lanelet.id]

    @property
    def shortest_relation_path_dict(self) -> dict[int, dict[int, list[int]]]:
        """All shortest relation paths from a given node to every node in a graph.

        Structured in a dictionary with lanelet id as keys and another inner dictionary as value.
        The inner dictionary holds the path represented by a list of roadgraph edge relations with
        the target lanelet being the key.
        Example:
        >>> {from_lanelet_id: {to_lanelet0_id: [path as a list], to_lanelet1_id: [path as a list]}}
        The paths are defined by edge tuples such as
        >>> (from_lanelet_id, "relation", weight: float, to_lanelet_id).
        If there is a path 1->2->3->4 and all relations were successors, the dict entry would be:
        >>> {1: {4: [(1, "Successor", <weight>, 2), (2, "Successor", <weight>, 3), (3, "Successor", <weight>, 4)]}}
        # noqa
        Returns:
            dict[int, dict[int, list[int]]]: All shortest paths structured in a dictionary.
        """
        return self.__shortest_relation_path_dict


class Relation(Enum):
    """An Enum for the possible Relations (INTERSECTING, LATERAL, LONGITUDINAL) between Entities.
    """

    INTERSECTING = 1
    LATERAL = 2
    LONGITUDINAL = 3


class EdgeMode(Enum):
    """An Enum for the possible modes for which edges in the SemanticSceneGraph are kept if there
    are double edges due to multiple ProjectionIdentities per Entity
    """

    SHORTEST = 1
    MOST_LIKELY = 2
    KEEP_ALL = 3
    NO_PARALLEL = 4


class SemanticSceneGraph:
    """Decorates igraph.Graph.
    the semantic scene graph model is utilized to describe traffic scenes in relation to road
    topology and to compare different scenes with each other, independent of the location.
    The relations between traffic participants are described by semantically classified edges.
    This abstract description facilitates machine readability and makes it practical to apply
    machine learning methods to traffic scenes.
    """

    def __init__(
        self,
        matching_dict: ProbabilisticMatchingDict,
        scene: csvloader.Scene,
        roadgraph: Roadgraph,
        projection_identity_dict: ProjectionIdentityDict,
        edge_mode=EdgeMode.MOST_LIKELY,
        verbose=False,
    ):
        if verbose:
            print("...............", matching_dict)
        self.__verbose = verbose
        self.__lanelet_dict = roadgraph._lanelet_dict
        self.__scene = scene
        self.__graph = igraph.Graph(directed=True)
        # Add nodes and their attributes to the scene graph
        self.__graph.add_vertices(len(matching_dict.keys()))
        self.__graph.vs["label"] = list(matching_dict.keys())
        for attribute_name in scene.entity_states[0].attribute_dict.keys():
            self.__graph.vs[attribute_name] = [
                scene.get_entity_state(
                    entity_id).attribute_dict[attribute_name]
                for entity_id in matching_dict.keys()
            ]
        # Add projection identity to the nodes as string
        self.__graph.vs["projection_identity"] = [
            str(projection_identity_dict[entity_id]).replace(
                "{", "").replace("}", "")
            for entity_id in matching_dict.keys()
        ]
        # Add edges. Three types of directed edges have to be added. All three types of edges
        # start at the vertex node of the relation path and end at the last vertex of the
        # relation path.
        # Let x ∈ ℕ0
        # A longitudinal type edge is added if the roadgraph relation path contains x consecutive
        # relations.
        # A lateral type edge is added if the roadgraph relation path contains x consecutive
        # relations and exactly one adjacent relation.
        # An intersecting type edge is added if the roadgraph relation path contins x consecutive
        # or adjacent relations and one overlapping relation. Note that once an edge with the
        # attribute overlapping is in path from a to b, all subsequent edges must be reversed.

        consecutive = "Successor"
        adjacent_left = "AdjacentLeft"
        adjacent_right = "AdjacentRight"
        overlapping = "Conflicting"
        # iterate over shortest relation paths of the directed roadgraph first
        for (
            origin_lanelet_id,
            target_lanelet_id_and_relation_path_dict,
        ) in roadgraph.shortest_relation_path_dict.items():
            for (
                target_lanelet_id,
                relation_path,
            ) in target_lanelet_id_and_relation_path_dict.items():
                relation_path_relations = [step[1] for step in relation_path]
                for origin_entity_id in projection_identity_dict.swap_keys[
                    origin_lanelet_id
                ].keys():
                    for target_entity_id in projection_identity_dict.swap_keys[
                        target_lanelet_id
                    ].keys():
                        if origin_entity_id != target_entity_id:
                            # intersecting
                            if relation_path_relations.count(overlapping) == 1:
                                path_distance = self.calculate_path_distance(
                                    origin_lanelet_id,
                                    target_lanelet_id,
                                    origin_entity_id,
                                    target_entity_id,
                                    relation_path,
                                    Relation.INTERSECTING,
                                )
                                if path_distance[1]:
                                    self.__graph.add_edges(
                                        [
                                            (
                                                self.__graph.vs.find(
                                                    id=origin_entity_id),
                                                self.__graph.vs.find(
                                                    id=target_entity_id),
                                            )
                                        ],
                                        attributes={
                                            "label": "int",
                                            "style": "dotted",
                                            "type": "intersecting",
                                            "path_distance": round(path_distance[0],
                                                                    DECIMAL_PLACES),
                                            "centerline_distance_origin": round(self.distance_from_centerline(  # noqa
                                                origin_entity_id, origin_lanelet_id),
                                                DECIMAL_PLACES),
                                            "centerline_distance_target": round(self.distance_from_centerline(  # noqa
                                                target_entity_id, target_lanelet_id),
                                                DECIMAL_PLACES),
                                            "probability": round(self.get_probability(
                                                origin_entity_id,
                                                origin_lanelet_id,
                                                target_entity_id,
                                                target_lanelet_id,
                                                projection_identity_dict),
                                                DECIMAL_PLACES),
                                            "from_lanelet": origin_lanelet_id,
                                            "to_lanelet": target_lanelet_id,
                                        },
                                    )
                            # lateral
                            elif (relation_path_relations.count(adjacent_left) == 1) ^ (
                                relation_path_relations.count(
                                    adjacent_right) == 1
                            ) and overlapping not in relation_path_relations:  # TODO: remove this line if tests work without it # noqa
                                path_distance = self.calculate_path_distance(
                                    origin_lanelet_id,
                                    target_lanelet_id,
                                    origin_entity_id,
                                    target_entity_id,
                                    relation_path,
                                    Relation.LATERAL,
                                )
                                if path_distance[1]:
                                    self.__graph.add_edges(
                                        [
                                            (
                                                self.__graph.vs.find(
                                                    id=origin_entity_id),
                                                self.__graph.vs.find(
                                                    id=target_entity_id),
                                            )
                                        ],
                                        attributes={
                                            "label": "lat",
                                            "style": "dashed",
                                            "type": "lateral",
                                            "path_distance": round(path_distance[0],
                                                                    DECIMAL_PLACES),
                                            "centerline_distance_origin": round(self.distance_from_centerline(  # noqa
                                                origin_entity_id, origin_lanelet_id),
                                                DECIMAL_PLACES),
                                            "centerline_distance_target": round(self.distance_from_centerline(  # noqa
                                                target_entity_id, target_lanelet_id),
                                                DECIMAL_PLACES),
                                            "probability": round(self.get_probability(
                                                origin_entity_id,
                                                origin_lanelet_id,
                                                target_entity_id,
                                                target_lanelet_id,
                                                projection_identity_dict),
                                                DECIMAL_PLACES),
                                            "from_lanelet": origin_lanelet_id,
                                            "to_lanelet": target_lanelet_id,
                                        },
                                    )
                            # longitudinal or Entities are on the same lanelet.
                            # This is also a longitudinal relation from the further back
                            # entity to the one in front.
                            elif (
                                consecutive in relation_path_relations
                                and adjacent_left not in relation_path_relations
                                and adjacent_right not in relation_path_relations
                                and overlapping
                                not in relation_path_relations  # TODO: remove this line if tests work without it # noqa
                            ) or (not relation_path and origin_lanelet_id == target_lanelet_id):
                                path_distance = self.calculate_path_distance(
                                    origin_lanelet_id,
                                    target_lanelet_id,
                                    origin_entity_id,
                                    target_entity_id,
                                    relation_path,
                                    Relation.LONGITUDINAL,
                                )
                                if path_distance[1]:
                                    self.__graph.add_edges(
                                        [
                                            (
                                                self.__graph.vs.find(
                                                    id=origin_entity_id),
                                                self.__graph.vs.find(
                                                    id=target_entity_id),
                                            )
                                        ],
                                        attributes={
                                            "label": "lon",
                                            "type": "longitudinal",
                                            "path_distance": round(path_distance[0],
                                                                    DECIMAL_PLACES),
                                            "centerline_distance_origin": round(self.distance_from_centerline(  # noqa
                                                origin_entity_id, origin_lanelet_id),
                                                DECIMAL_PLACES),
                                            "centerline_distance_target": round(self.distance_from_centerline(  # noqa
                                                target_entity_id, target_lanelet_id),
                                                DECIMAL_PLACES),
                                            "probability": round(self.get_probability(
                                                origin_entity_id,
                                                origin_lanelet_id,
                                                target_entity_id,
                                                target_lanelet_id,
                                                projection_identity_dict),
                                                DECIMAL_PLACES),
                                            "from_lanelet": origin_lanelet_id,
                                            "to_lanelet": target_lanelet_id,
                                        },
                                    )
                            # This shouldn't really happen.
                            # But if it does, print the path as an error.
                            elif relation_path and verbose:
                                print("Invalid path: " + str(relation_path))

        edge_count = len(self.__graph.es)
        self.remove_double_edges(self.__graph, edge_mode)
        if verbose:
            print(
                f"There are now {len(self.__graph.es)} edges left in the graph. \
                {edge_count-len(self.__graph.es)} double edges have been removed \
                because of the edge filter mode {edge_mode}."
            )

    def remove_double_edges(self, graph: igraph.Graph, edge_mode: EdgeMode):
        """Removes all double edges from the given graph according to the edge mode
        The edges have to have the attributes "probability" and "path_distance"

        Args:
            graph (igraph.Graph): The graph from which the edges are to be removed
            edge_mode (EdgeMode): The mode of the edge removal process
        """
        if edge_mode != EdgeMode.KEEP_ALL and len(graph.es) > 0:  # noqa
            if edge_mode == EdgeMode.NO_PARALLEL:
                edge_dict = defaultdict(list)
            else:
                edge_dict = {}
            for edge in graph.es:
                origin_entity_id = edge.tuple[0]
                target_entity_id = edge.tuple[1]
                if (edge_mode == EdgeMode.NO_PARALLEL):
                    edge_dict[(origin_entity_id, target_entity_id)
                              ].append(edge)
                    edge["type"] = "deletion"
                elif (origin_entity_id, target_entity_id) in edge_dict:
                    other_edge = edge_dict[(
                        origin_entity_id, target_entity_id)]
                    # Edge is double, check according to edge_mode and remove one of them
                    if (
                        edge_mode == EdgeMode.MOST_LIKELY
                        and edge["probability"] > other_edge["probability"]
                    ) or (
                        edge_mode == EdgeMode.SHORTEST
                        and edge["path_distance"] < other_edge["path_distance"]
                    ):
                        edge_dict[(origin_entity_id, target_entity_id)] = edge
                        other_edge["type"] = "deletion"
                    else:
                        edge["type"] = "deletion"
                else:
                    edge_dict[(origin_entity_id, target_entity_id)] = edge

            if (edge_mode == EdgeMode.NO_PARALLEL):
                for key, edge_list in edge_dict.items():
                    probability_int = 0.0
                    probability_lat = 0.0
                    probability_lon = 0.0
                    shortest_path_distance = 10e9
                    shortest_int_distance = 10e9
                    centerline_distance_origin = 10e9
                    centerline_distance_target = 10e9
                    int_centerline_distance_origin = 10e9
                    int_centerline_distance_target = 10e9
                    for tmp_edge_el in edge_list:
                        if tmp_edge_el["label"] == "lon":
                            probability_lon += tmp_edge_el["probability"]
                            if shortest_path_distance > tmp_edge_el["path_distance"]:  # noqa
                                shortest_path_distance = tmp_edge_el["path_distance"]
                                centerline_distance_origin = tmp_edge_el["centerline_distance_origin"]  # noqa
                                centerline_distance_target = tmp_edge_el["centerline_distance_target"]  # noqa
                        if tmp_edge_el["label"] == "lat":
                            probability_lat += tmp_edge_el["probability"]
                            if shortest_path_distance > tmp_edge_el["path_distance"]:  # noqa
                                shortest_path_distance = tmp_edge_el["path_distance"]
                                centerline_distance_origin = tmp_edge_el["centerline_distance_origin"]  # noqa
                                centerline_distance_target = tmp_edge_el["centerline_distance_target"]  # noqa
                        if tmp_edge_el["label"] == "int":
                            probability_int += tmp_edge_el["probability"]
                            if shortest_int_distance > tmp_edge_el["path_distance"]:  # noqa
                                shortest_int_distance = tmp_edge_el["path_distance"]
                                int_centerline_distance_origin = tmp_edge_el["centerline_distance_origin"]  # noqa
                                int_centerline_distance_target = tmp_edge_el["centerline_distance_target"]  # noqa
                        # get shortest distance:

                    shortest_path_distance = 0 if shortest_path_distance > 10e8 else shortest_path_distance  # noqa
                    centerline_distance_origin = 0 if centerline_distance_origin > 10e8 else centerline_distance_origin  # noqa
                    centerline_distance_target = 0 if centerline_distance_target > 10e8 else centerline_distance_target  # noqa
                    int_centerline_distance_origin = 0 if int_centerline_distance_origin > 10e8 else int_centerline_distance_origin  # noqa
                    int_centerline_distance_target = 0 if int_centerline_distance_target > 10e8 else int_centerline_distance_target  # noqa
                    shortest_int_distance = 0 if shortest_int_distance > 10e8 else shortest_int_distance  # noqa

                    graph.add_edges([key], attributes={
                        "label": "merged",
                        "style": "None",
                        "type": "merged",
                        "path_distance": shortest_path_distance,
                        "path_int_distance": shortest_int_distance,
                        "centerline_distance_origin": centerline_distance_origin,
                        "centerline_distance_target": centerline_distance_target,
                        "int_centerline_distance_origin": int_centerline_distance_origin,
                        "centerline_distance_target": centerline_distance_target,
                        "int_centerline_distance_target": int_centerline_distance_target,
                        "probability_lon": probability_lon,
                        "probability_lat": probability_lat,
                        "probability_int": probability_int,
                        # "from_lanelet": origin_lanelet_id,
                        # "to_lanelet": target_lanelet_id,
                    },
                    )

            # We have to remove edges like this here because removing edges while storing other
            # edge objects outside of the graph messes with the edges
            graph.delete_edges(graph.es.select(type_eq="deletion"))

    def calculate_path_distance(
        self,
        origin_lanelet_id,
        target_lanelet_id,
        origin_entity_id,
        target_entity_id,
        relation_path: dict,
        relation: Relation,
    ):
        """Calculates the distance between two entities along their relation path.
        If the relation is "intersecting", the distance is only measured between the origin entity
        and the intersection point.

        Args:
            origin_lanelet_id (int): the lanelet id of the origin lanelet
            target_lanelet_id (int): the lanelet id of the target lanelet
            origin_entity_id (int): the id of the origin entity
            target_entity_id (int): the id of the target entity
            relation_path (dict): the path between the entities
            relation (Relation): the relation of the path

        Returns:
            float: the distance between the entities, negative if the path is not valid.
        """
        distance = 0.0
        path_is_valid = True
        # determine the lanelet the origin entity is on
        lanelet = self.__lanelet_dict[origin_lanelet_id]

        origin_entity_state = self.__scene.get_entity_state(origin_entity_id)
        target_entity_state = self.__scene.get_entity_state(target_entity_id)

        # determine location of target entity
        target_entity_location = lanelet2.core.BasicPoint3d(
            target_entity_state.x,
            target_entity_state.y,
        )
        target_lanelet = self.__lanelet_dict[target_lanelet_id]
        target_entity_location_arc = lanelet2.geometry.toArcCoordinates(
            lanelet2.geometry.to2D(target_lanelet.centerline),
            lanelet2.geometry.to2D(target_entity_location),
        )

        # the current point in the measurement process
        current_measurement_point = lanelet2.core.BasicPoint3d(
            origin_entity_state.x,
            origin_entity_state.y,
            0,
        )

        if not relation_path:
            # relation path is empty --> origin and target Entity on same lanelet
            current_measurement_location_arc = lanelet2.geometry.toArcCoordinates(
                lanelet2.geometry.to2D(lanelet.centerline),
                lanelet2.geometry.to2D(current_measurement_point),
            )
            # negative if the target is behind the origin.
            # This means there is no relation origin --Successor--> target
            distance = target_entity_location_arc.length - \
                current_measurement_location_arc.length
            if distance < 0:
                path_is_valid = False
            return (distance, path_is_valid)

        for step in relation_path:
            lanelet = self.__lanelet_dict[step[0]]
            next_lanelet = self.__lanelet_dict[step[-1]]
            current_measurement_location_arc = lanelet2.geometry.toArcCoordinates(
                lanelet2.geometry.to2D(lanelet.centerline),
                lanelet2.geometry.to2D(current_measurement_point),
            )

            if step[1] == "Conflicting":
                # Calculate distance to intersection
                if relation != relation.INTERSECTING and self.__verbose:
                    # TODO: remove this and refactor everything in the for loop of the constructor
                    # TODO: remove the relation enum.
                    print(
                        "Intersecting relation not classified as one but as " + str(relation))
                intersection_points = lanelet2.geometry.intersectCenterlines2d(
                    lanelet, next_lanelet
                )
                if not intersection_points:
                    # No intersection of centerlines, calculate distance to the point where the
                    # centerlines are closest to each other instead
                    intersection_points = lanelet2.geometry.projectedPoint3d(
                        lanelet.centerline, next_lanelet.centerline
                    )
                intersection_location_arc = lanelet2.geometry.toArcCoordinates(
                    lanelet2.geometry.to2D(lanelet.centerline),
                    lanelet2.geometry.to2D(intersection_points[0]),
                )

                # Check if the current measurement point has already passed the first#
                # intersecting location
                if (
                    current_measurement_location_arc.length
                    > intersection_location_arc.length + origin_entity_state.length / 2
                ):
                    # Set intersecting location to last intersecting location
                    intersection_location_arc = lanelet2.geometry.toArcCoordinates(
                        lanelet2.geometry.to2D(lanelet.centerline),
                        lanelet2.geometry.to2D(intersection_points[-1]),
                    )
                    if (
                        current_measurement_location_arc.length
                        > intersection_location_arc.length + origin_entity_state.length / 2
                    ):
                        if self.__verbose:
                            print(
                                "Origin Entity has already passed the intersecting location.\
                                 The current relation is not a valid relation and has to be removed."  # noqa
                                + str(step)
                                + str(origin_entity_id)
                                + ", "
                                + str(target_entity_id)
                            )
                        path_is_valid = False
                        return (distance, path_is_valid)

                next_intersection_location_arc = lanelet2.geometry.toArcCoordinates(
                    lanelet2.geometry.to2D(next_lanelet.centerline),
                    lanelet2.geometry.to2D(intersection_points[0]),
                )

                # TODO: check if target entity that is at least one lane change away from
                # conflicting relation has passed the intersecting point
                if (
                    target_lanelet_id == step[-1]
                    and target_entity_location_arc.length
                    > next_intersection_location_arc.length + target_entity_state.length / 2
                ):
                    # Entity has passed the first intersecting location. Set intersecting location
                    # to last intersecting location
                    intersection_location_arc = lanelet2.geometry.toArcCoordinates(
                        lanelet2.geometry.to2D(lanelet.centerline),
                        lanelet2.geometry.to2D(intersection_points[-1]),
                    )
                    next_intersection_location_arc = lanelet2.geometry.toArcCoordinates(
                        lanelet2.geometry.to2D(next_lanelet.centerline),
                        lanelet2.geometry.to2D(intersection_points[-1]),
                    )
                    if (
                        current_measurement_location_arc.length
                        > next_intersection_location_arc.length + target_entity_state.length / 2
                    ):
                        if self.__verbose:
                            print(
                                "Target Entity has already passed the intersecting location. \
                                The current relation isnot a valid relation and has to be removed."
                                + str(step)
                                + str(origin_entity_id)
                                + ", "
                                + str(target_entity_id)
                            )
                        path_is_valid = False
                        return (distance, path_is_valid)

                distance = distance + (
                    intersection_location_arc.length - current_measurement_location_arc.length
                )
                return (distance, path_is_valid)

            elif step[1] == "AdjacentLeft" or step[1] == "AdjacentRight":
                # Calculate distance to lane change and distance to other Lanelet's centerline
                # Do the lane change asap and not at the closest point because it can cause
                # problems with multiple lane changes in a row
                current_measurement_point = lanelet2.geometry.project(
                    next_lanelet.centerline, current_measurement_point
                )
                # the arc coordinates of the point that has been projected on the next lanelet
                # but in the context of the current lanelet
                projected_arc_coordinates = lanelet2.geometry.toArcCoordinates(
                    lanelet2.geometry.to2D(lanelet.centerline),
                    lanelet2.geometry.to2D(current_measurement_point),
                )

                distance = distance + projected_arc_coordinates.distance

            elif step[1] == "Successor":
                # Calculate length of current lanelet - the part of the lanelet that is already
                # behind the current measurement location
                distance = distance + (
                    lanelet2.geometry.length2d(
                        lanelet) - current_measurement_location_arc.length
                )
                # set the current measurement location to the start of the next lanelet
                current_measurement_point = next_lanelet.centerline[0]
                lanelet = next_lanelet

            # convert current_measurement_point because lanelet will otherwise cause problems
            current_measurement_point = lanelet2.core.BasicPoint3d(
                current_measurement_point.x,
                current_measurement_point.y,
                0,
            )

        current_measurement_location_arc = lanelet2.geometry.toArcCoordinates(
            lanelet2.geometry.to2D(lanelet.centerline),
            lanelet2.geometry.to2D(current_measurement_point),
        )

        if (
            self.__verbose
            and current_measurement_location_arc.length > target_entity_location_arc.length
        ):
            # the origin entity is actually in front of the target entity
            #   --> relation might not be valid.
            print(
                "target is adjacent and behind origin. Distance is: "
                + str(distance)
                + ". Relation is "
                + str(relation)
            )

        if (
            current_measurement_location_arc.length > target_entity_location_arc.length
            and relation == Relation.LATERAL
        ):
            distance = target_entity_location_arc.length - \
                current_measurement_location_arc.length
        else:
            distance = abs(
                distance
                + target_entity_location_arc.length
                - current_measurement_location_arc.length
            )

        return (distance, path_is_valid)

    def distance_from_centerline(self, entity_id, entity_lanelet) -> float:
        """Returns the distance of the Entity from the centerline of the Lanelet.

        Args:
            entity_id (int): The id of the Entity.
            entity_lanelet (int): The id of the Lanelet.

        Returns:
            float: The calculated distance between Entity and the centerline of the Lanelet.
        """
        return lanelet2.geometry.toArcCoordinates(
            lanelet2.geometry.to2D(
                self.__lanelet_dict[entity_lanelet].centerline),
            lanelet2.geometry.to2D(
                lanelet2.core.BasicPoint3d(
                    self.__scene.get_entity_state(entity_id).x,
                    self.__scene.get_entity_state(entity_id).y,
                    0,
                )
            ),
        ).distance

    def get_probability(
        self,
        origin_entity_id,
        origin_lanelet_id,
        target_entity_id,
        target_lanelet_id,
        projection_identity_dict,
    ) -> float:
        """Calculates the probability of the given edge based on the probabilities of the matching
        identities of the two matches the edge connects.
        The matches of the Entities on the provided Lanelets has to actually exist.

        Args:
            origin_entity_id (int): The id of the origin entity
            origin_lanelet_id (int): The id of the lanelet the origin Projection Identity is on
            target_entity_id (int): The id of the target entity
            target_lanelet_id (int): The id of the lanelet the target Projection Identity is on
            projection_identity_dict (ProjectionIdentityDict): The calculated ProjectionIdentityDict
            # noqa

        Returns:
            float: the probability of the edge
        """
        return (
            projection_identity_dict[origin_entity_id][origin_lanelet_id]
            * projection_identity_dict[target_entity_id][target_lanelet_id]
        )

    def __iter__(self):
        return self.__dict__["_SemanticSceneGraph__graph"].__iter__()

    def __next__(self):
        return self.__dict__["_SemanticSceneGraph__graph"].__next__()

    def __getattr__(self, name):
        return getattr(self.__dict__["_SemanticSceneGraph__graph"], name)

    def __delattr__(self, name):
        delattr(self.__dict__["_SemanticSceneGraph__graph"], name)


def read_to_lanelet_map(
    osm_path: str, origin: tuple[float, float], crop_box=None, verbose=False
) -> lanelet2.core.LaneletMap:
    """Conveniently creates a LaneletMap object from .osm file.

    Args:
        osm_path (str): file path to the .osm file
        origin (tuple[float, float]): coordinates to the road segment
            (must have a high level of accuracy)
        crop_box (List): x_max, x_min, y_max, y_max for cropping large lanetlet maps
        verbose (bool, optional): Set to True for additional output. Defaults to False.
    """
    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(origin[0], origin[1]))
    lanelet_map, error_list = lanelet2.io.loadRobust(osm_path, projector)
    if verbose:
        print(
            f"{len(error_list)} errors, {len([lane for lane in lanelet_map.laneletLayer])} lanes")

    if crop_box is not None:
        max_x = crop_box[0]
        min_x = crop_box[1]
        max_y = crop_box[2]
        min_y = crop_box[3]
        cropped_lanelet = lanelet_map.laneletLayer.search(lanelet2.core.BoundingBox2d(
            lanelet2.core.BasicPoint2d(min_x, min_y), lanelet2.core.BasicPoint2d(max_x, max_y)))
        lanelet_map = lanelet2.core.createMapFromLanelets(cropped_lanelet)

    return lanelet_map
