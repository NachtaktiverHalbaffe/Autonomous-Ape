#!/usr/bin/env python3
import signal
import subprocess


def start_launch_file(
    ros2_package: str, launch_file: str, arguments: str = ""
) -> subprocess.Popen:
    cmd = f"ros2 launch {ros2_package} {launch_file} {arguments}"
    return subprocess.Popen(cmd.split(" "))


def shutdown_nodes_launch_file(shell_process: subprocess.Popen | None) -> bool:
    """
    Shutdown a process which runs over a shell i.e. was called with `subprocess.Popen()`

    Args:
        shell_process (Popen or None): The shell process which should be should down

    Returns:
        bool: Indicating if process was shutdown successfully or not
    """
    if shell_process is not None:
        shell_process.send_signal(signal.SIGINT)
        shell_process.wait(timeout=10)
        shell_process = None
        return True
    else:
        return False


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
