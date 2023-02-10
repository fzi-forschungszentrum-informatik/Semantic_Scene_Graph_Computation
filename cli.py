"""This is a module that acts as a Command Line Interface (CLI)
for the semantic-scene-graph project
"""

import time
import argparse
import pandas as pd
from controller import Controller

ALIAS_FILE_PATH = "cli_aliases.csv"


def call_controller(
    csv_path: str, osm_path: str, x_origin, y_origin, timestamps,
    clean_load, verbose=False, outputfolder="./dotgraphs"
):
    """Calls the init function of controller.Controller and starts the program by invoking
    its main function passing the provided parameters.Times the whole process and prints the time.

    Args:
        csv_path (str): The path of the csv file containing the traffic data.
        osm_path (str): The path of the osm file containing the street data.
        x_origin (float): The x coordinate of the origin of the used dataset.
        y_origin (float): The y coordinate of the origin of the used dataset.
        timestamps (list(int)): The timestamps to be computed. If an empty list is provided,
            all timestamps of the dataset are computed.
        clean_load (bool): If the Loader module should reload the object structure from the csv
            file if there is an existing pickle.
        verbose (bool, optional): If verbose feedback is wanted. Defaults to False.
    """
    origin = (x_origin, y_origin)
    if verbose:
        print(f"Loading: {csv_path} {osm_path} {origin} verbose={verbose}")

    controller = Controller()

    tic = time.perf_counter()
    controller.main(csv_path, osm_path, origin, timestamps,
                    clean_load, verbose, outputfolder=outputfolder)
    toc = time.perf_counter()

    print(f"Ran everything in {toc- tic:0.4f} seconds")


class ListAliasesAction(argparse.Action):
    """Class for the list_aliases option.
    Extends argparse.Action.
    """

    def __init__(self, option_strings, dest, **kwargs):
        """A simple call to the super init.

        Args:
            see super().__init__
        """
        super().__init__(option_strings, dest, nargs=0, default=argparse.SUPPRESS, **kwargs)

    def __call__(self, parser, namespace, values, option_string, **kwargs):
        """Runs the list_aliases action. Reads all saved aliases from the csv file at
        ALIAS_FILE_PATH and prints them and their parameters.
        Exits the parser in the end. Ignores every argument except --list_aliases.

        Args:
            parser (argparse.ArgumentParser): The used parser.
            namespace (argparse.Namespace): The namespace containing all used arguments.
            values (list): The list of values.
            option_string (str): The string of the option. Should be "--list_aliases" always
        """
        # Read and print the aliases
        print("The defined aliases are:")
        df = pd.read_csv(ALIAS_FILE_PATH)
        if not df.empty:
            print(df.to_string(index=False))
        else:
            print("There are no saved aliases yet.")

        parser.exit()


class UseAliasAction(argparse.Action):
    """Extends argparse.Action. Action class for the --alias/-a ALIAS TIMESTAMPS argument."""

    def __init__(self, option_strings, dest, **kwargs):
        """Calls the super init.

        Args:
            see super().__init__
        """
        super(UseAliasAction, self).__init__(
            option_strings, dest, nargs="+", default=argparse.SUPPRESS, **kwargs
        )

    def __call__(self, parser, namespace, values, option_string, **kwargs):
        """Runs the alias action. Loads a saved aliases from the csv file at ALIAS_FILE_PATH and
        runs call_controller() with its parameters.
        Exits the parser in the end. Ignores every argument except --alias (-a) and the
        timestamps after it.

        Args:
            parser (argparse.ArgumentParser): The used parser.
            namespace (argparse.Namespace): The namespace containing all used arguments.
            values (list): The list of values. Here the alias string followed by the timestamps.
            option_string (str): The string of the option. Should be "--list_aliases" always
        """
        alias = values.pop(0)
        timestamps = list(map(int, values))

        # look for the alias in the csv file and run call_controller
        df = pd.read_csv(ALIAS_FILE_PATH)
        alias_row = df.loc[df["alias"] == alias]
        if not alias_row.empty:
            call_controller(
                alias_row["csv_path"].values[0],
                alias_row["osm_path"].values[0],
                alias_row["x_origin"].values[0],
                alias_row["y_origin"].values[0],
                timestamps,
                namespace.clean_load,
                namespace.verbose,
                alias_row["outputfolder"].values[0]
            )
        else:
            print(f'The provided alias "{alias}" does not exist')
        parser.exit()


class Formatter(argparse.HelpFormatter):
    """Formatter for displaying the timestamp option after the --alias/-a argument correctly.
    Extends argparse.HelpFormatter
    """

    def _format_args(self, action: argparse.Action, default_metavar: str) -> str:
        """Formats the help output for the --alias/-a argument correctly.

        Args:
            action (argparse.Action): The action this formatter is used on. Only changes the
                return if the alias action is provided.
            default_metavar (str): The default metavar. See super()._format_args.

        Returns:
            str: calls the super function if the action is not alias. If it is it returns
                the formatted help string.
        """
        get_metavar = self._metavar_formatter(action, default_metavar)
        if action.nargs == argparse.ONE_OR_MORE and action.dest == "alias":
            return f"{get_metavar(1)[0]} [timestamps ...]"
        else:
            return super(Formatter, self)._format_args(action, default_metavar)


if __name__ == "__main__":
    # Create a parser and adds all required arguments
    parser = argparse.ArgumentParser(
        formatter_class=Formatter,
        description="Create a Semantic Scene Graph from a dataset in the TAF or the inD format.",
    )
    parser.add_argument(
        "csv_path",
        help="the csv file location. The csv file can either be formatted in the TAF format or in the inD format.",  # noqa
    )
    parser.add_argument(
        "osm_path",
        help="the osm file location. The osm file describes the map of the traffic scenario",
    )
    parser.add_argument(
        "x_origin", type=float, help="the x value of the origin of the provided dataset"
    )
    parser.add_argument(
        "y_origin", type=float, help="the y value of the origin of the provided dataset"
    )
    parser.add_argument(
        "timestamps",
        nargs="*",
        type=int,
        help="all timestamps that are supposed to be calculated. Default if left empty is all timestamps in the csv file.",  # noqa
        default=[],
    )
    parser.add_argument(
        "-of", "--outputfolder",
        type=str,
        help="outputfolder name. default .dotsgraphs",  # noqa
        default="./dotgraphs",
    )
    parser.add_argument(
        "-v", "--verbose", help="activates verbose cli feedback", action="store_true"
    )
    parser.add_argument(
        "--list_aliases",
        help="lists all saved locations and their aliases",
        action=ListAliasesAction,
    )
    parser.add_argument(
        "--save_alias",
        type=str,
        help="save the given csv, osm and origin information so it can be used with the alias later",  # noqa
        metavar="ALIAS",
    )
    parser.add_argument(
        "-c",
        "--clean_load",
        help="loads the csv file again if it already has been loaded",
        action="store_true",
    )
    parser.add_argument(
        "-a",
        "--alias",
        metavar="ALIAS",
        type=str,
        help="loads the dataset of the given alias and creates its Semantic Scene Graphs \
            for either the provided timestamps or the whole scenario. This option has to be \
                 the last option if it is used.",
        action=UseAliasAction,
    )
    # Parse the provided arguments
    args = parser.parse_args()

    # Check if the save_alias action is called and writes the alias and the parameters to the
    # csv file at ALIAS_FILE_PATH
    if args.save_alias is not None:
        df = pd.read_csv(ALIAS_FILE_PATH)
        alias_column = df["alias"]
        if args.save_alias not in alias_column.values:
            df = pd.DataFrame(
                {
                    "alias": [args.save_alias],
                    "csv_path": [args.csv_path],
                    "osm_path": [args.osm_path],
                    "x_origin": [args.x_origin],
                    "y_origin": [args.y_origin],
                    "outputfolder": [args.outputfolder],
                }
            )
            df.to_csv(ALIAS_FILE_PATH, mode="a", header=False, index=False)
        else:
            print(
                "The provided alias has already been assigned. Please use a new alias and try again."  # noqa
            )
            parser.exit()

    # Run call controller with all the given parameters
    call_controller(
        args.csv_path,
        args.osm_path,
        args.x_origin,
        args.y_origin,
        args.timestamps,
        args.clean_load,
        args.verbose,
        args.outputfolder,
    )
