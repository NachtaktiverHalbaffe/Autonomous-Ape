"""
This scripts automatically generates the Python Ui Object from a QT Designer Ui file. For running, use
"""

import getopt
import os.path
import subprocess
import sys


# Generate Sphinx files (Python)
def generate_ui_object(
    path_ui_file: str, path_dst: str, file_name: str = "ui_object.py"
):
    subprocess.call(
        [
            "python3",
            "-m",
            "PyQt5.uic.pyuic",
            path_ui_file,
            "-o",
            f"{path_dst}/{file_name}",
        ]
    )
    subprocess.call(["colcon", "build"])


if __name__ == "__main__":
    inputfile = ""
    outputfile = ""
    file_name = None

    try:
        opts, args = getopt.getopt(
            sys.argv[1:], "hi:o:n:", ["ifile=", "ofile=", "name"]
        )
    except getopt.GetoptError:
        print(
            "generate_ui.py -i <inputfile> -o <path of outputfile> -n <name of outputfile>"
        )
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-h":
            print(
                "generate_ui.py -i <inputfile> -o <path of outputfile> -n <name of outputfile>"
            )
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ifile"):
            outputfile = arg
        elif opt in ("-n", "--name"):
            file_name = arg

    if not os.path.isfile(inputfile):
        print(f"Input file {inputfile} not found")
        sys.exit()
    elif not os.path.exists(outputfile):
        print(f"Output path {outputfile} for generated ui file doesn't exist")
        sys.exit()

    if file_name is not None:
        generate_ui_object(inputfile, outputfile, file_name)
    else:
        generate_ui_object(inputfile, outputfile)
