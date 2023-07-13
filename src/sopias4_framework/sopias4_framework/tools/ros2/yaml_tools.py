import os
import sys

import ruamel.yaml
import ruamel.yaml.comments
import ruamel.yaml.parser


def insert_namespace_into_yaml_config(
    namespace: str, path: str, output_path: str
) -> bool:
    """
    Adds a namesapce into a configuration file which has to be in a yaml format (Default in ROS2).

    Args:
        namespace (str): The namespace which should be inserted
        path (str): Path to the Yaml file. Can be either relative or absolute, but absolute paths are safer
        output_path (str): Path where the file should be saved. Can be either relative or absolute, but absolute paths are safer

    Returns:
        bool: Indicates if namespace was added successfully
    """
    try:
        yaml_file = load_yaml(path=path)
    except Exception as e:
        print(
            f"Couldn't insert namespace into yaml configuration file because file couldn't be loaded: {e}"
        )
        return False
    yaml_file = insert_namespace(yaml_file, namespace)

    try:
        save_yaml(yaml_file, output_path)
        return True
    except Exception as e:
        print(
            f"Couldn't insert namespace into yaml configuration file because file couldn't be saved: {e}"
        )
        return False


def load_yaml(path: str) -> ruamel.yaml.CommentedMap:
    """
    Loads a yaml file from a given path

    Args:
        path (str): Path to the Yaml file. Can be either relative or absolute,
                          but absolute paths are safer

    Returns:
        ruamel.yaml.CommentedMap: The loaded Yaml file

    Raises:
        FileNotFoundError: When path top file is invalid
        ValueError: When the given file isn't a valid YAML file
    """
    yaml = ruamel.yaml.YAML()
    data: ruamel.yaml.CommentedMap | None = None

    # Validate that file exists
    if not os.path.isfile(path):
        raise FileNotFoundError(f"File path: {path} doesn't point to an existing file")

    with open(path) as fp:
        try:
            data = yaml.load(fp)
        except ruamel.yaml.parser.ParserError:
            raise ValueError(f"File, located under {path}, isn't a valid yaml file")

    return data


def save_yaml(yaml: ruamel.yaml.CommentedMap, output_path: str) -> None:
    """
    Saves a (modified) yaml instance to an file

    Args:
        yaml (ruamel.yaml.CommentedMap): The yaml instance which should be saved
        output_path (str): Path where the file should be saved

    Returns:
        bool: If file was saved successfully
    """
    yaml_handler: ruamel.yaml.YAML = ruamel.yaml.YAML()

    try:
        with open(output_path, "w") as fp:
            try:
                yaml_handler.dump(yaml, fp)
            except IOError as e:
                raise IOError(
                    f"Couldn't save yaml file to path {output_path}: Directory doesn't exist"
                )
    except Exception as e:
        raise e


def insert_namespace(
    yaml: ruamel.yaml.CommentedMap, namespace: str
) -> ruamel.yaml.CommentedMap:
    """
    Inserts the namespace to all neccessary elements of an ROS2 yaml config file. Currently supports
    config files for nav2, slam and amcl.

    Args:
        yaml (ruamel.yaml.CommentedMap): The yaml instance which should be modified
        output_path (str): Path where the file should be saved

    Returns:
        ruamel.yaml.CommentedMap: A modified yaml instance with all namespaces inserted
    """
    modified_yaml = yaml

    for key in modified_yaml.keys():
        modified_yaml = __iterate_through_file(modified_yaml, key, namespace)

    return modified_yaml


def __iterate_through_file(
    yaml_instance: ruamel.yaml.CommentedMap,
    item_key: str,
    namespace: str,
) -> ruamel.yaml.CommentedMap:
    """
    Iterates through the yaml file to get the deepest nested items and adds a namespace to them if necessary.\
    This method calls itself recursively until the deepest nested items is reached

    Args:
        yaml_instance (ruamel.yaml.CommentedMap): The yaml instance into which the namespace should be added
        item_key (str): The current key of the nested item which is visited during this recursice_call. If called first time, then give \
                                 key of top level item. Each nested level is seperated by a sep-symbol (defaults to "."), so a valid key looks like\
                                 e.g. "first_element.nested_element.even_more_nested_element"
        namespace (str): The namespace which should be inserted

    Returns:
        ruamel.yaml.CommentedMap: A modified yaml_instance into which the namespace was added
    """
    for attr in yaml_instance.mlget(item_key.split(".")).items():
        # print(type(attr[1]))
        if type(attr[1]) is not ruamel.yaml.CommentedMap:
            yaml_instance = __add_namespace_to_item(
                yaml_instance, f"{item_key}.{attr[0]}", namespace
            )
        if type(attr[1]) is ruamel.yaml.CommentedMap:
            yaml_instance = __iterate_through_file(
                yaml_instance, f"{item_key}.{attr[0]}", namespace
            )

    return yaml_instance


def __add_namespace_to_item(
    yaml_instance: ruamel.yaml.CommentedMap, item_key: str, namespace: str
) -> ruamel.yaml.CommentedMap:
    """
    Adds the namespace to the neccessary items which can be deeply nested into a yaml file and\
    needs a correct namespace for working correctly

    Args:
        yaml_instance (ruamel.yaml.CommentedMap): The yaml instance in which the item is modified
        item_key (str): The key of the item. Each nested level is seperated by a sep-symbol (defaults to "."), so a valid\
                                 key looks like e.g. "first_element.nested_element.even_more_nested_element"
        namespace (str): The namespace which should be added

    Returns:
        ruamel.yaml.CommentedMap: A yaml_instance in which the namespace was added
    """
    last_key = item_key.split(".")[-1:][0]
    if last_key in ["topic", "scan_topic"]:
        value = f"{namespace}/{yaml_instance.mlget(item_key.split('.'))}"
        # Add leading / to namespace if it wasnt given in the argument
        if namespace[0] != "/":
            value = f"/{value}"
        yaml_instance.mlput(path=item_key, value=value)
    return yaml_instance


def __mlput(
    self: ruamel.yaml.CommentedMap,
    path: str,
    value,
    sep: str = ".",
) -> None:
    """
    This method allows us to modify deeply nested items in a ruaml.yaml.Commentedmap item.

    Args:
        path (str): The key of the item. Each nested level is seperated by a sep-symbol (defaults to "."), so a valid key \
                          looks like e.g. "first_element.nested_element.even_more_nested_element"
        value (any): The value that should be inserted
        sep (str, optional): The seperator symbol that seperates each key of each nested level. Defaults to "."
    """
    spath = path.split(sep)
    parent: ruamel.yaml.CommentedMap = self.mlget(spath[:-1])
    parent[spath[-1]] = value


# Insert mlput method into ruamel.yaml.CommentedMap type so it usable
ruamel.yaml.CommentedMap.mlput = __mlput

if __name__ == "__main__":
    print(
        "Only run this if used for testing or manual operation. It was designed to be used inside a program which handles launching a ROS2 launch file with config parameters"
    )
    insert_namespace_into_yaml_config(
        "ape",
        "/home/ws/src/sopias4_framework/config/nav2_base.yaml",
        "/home/ws/src/sopias4_framework/config/nav2.yaml",
    )
