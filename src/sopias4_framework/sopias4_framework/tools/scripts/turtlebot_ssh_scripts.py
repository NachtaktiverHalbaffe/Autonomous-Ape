"""
This module provides different scripts which executes commands remotely on the turtlebot via SSH.
"""

import getopt
import sys

import paramiko


def send_ssh_cmd(
    ip_adress: str, command: str, port: int = 22, password: str = "turtlebot4"
) -> None:
    """
    Executes a command remotely on the turtlebot. For this purpose, a SSH connection is established and the necessary command is run

    Args:
        ip_adress (str): The ipv4 adress of the Raspberry Pi of the Turtlebot
        port (int, optional): The SSH port. Defaults to 22 (standard SSH port)
        password (str, optional): The used password for connecting via SSH. Defaults to the default password, thus only needed to be changed if password was changed on the Raspberry Pi
    """
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print("Connecting to to Turtlebot ....")
    # Connecting with credentials
    try:
        ssh.connect(
            hostname=ip_adress,
            port=port,
            username="ubuntu",
            password=password,
            timeout=10.0,
        )

    except paramiko.AuthenticationException:
        print(
            f"Couldn't connect via SSH because the credentials (username: ubuntu, pw: {password}) for host {ip_adress} was invalid"
        )
        return
    except Exception as e:
        print(f"Couldnt connect via SSH to {ip_adress}: {e}")
        return
    print("Connected!")

    print(f"Executing command {command} ...")
    # Execute command
    try:
        ssh.get_transport().open_session().exec_command(command)  # type: ignore
    except paramiko.SSHException:
        print(f"Couldn't execute the ssh command {command} via SSH on host {ip_adress}")
        return
    except Exception as e:
        print(
            f"Couldn't execute the ssh command {command}via SSH: Error {e} on client side"
        )
    finally:
        ssh.close()

    print("Executed command! The nodes should show up in a few seconds")


def restart_turtlebot(
    ip_adress: str, port: int = 22, password: str = "turtlebot4"
) -> None:
    """
    Restarts the turtlebot4 nodes. For this purpose, a SSH connection is established and the necessary command is run

    Args:
        ip_adress (str): The ipv4 adress of the Raspberry Pi of the Turtlebot
        port (int, optional): The SSH port. Defaults to 22 (standard SSH port)
        password (str, optional): The used password for connecting via SSH. Defaults to the default password, thus only needed to be changed if password was changed on the Raspberry Pi
    """
    send_ssh_cmd(
        ip_adress=ip_adress,
        command="turtlebot4-service-restart",
        port=port,
        password=password,
    )


if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(
            sys.argv[1:], "p:a:c:", ["port", "ipv4-addr", "command"]
        )
    except getopt.GetoptError:
        print(
            'python3 restart_turtlebot_nodes.py -a <ipv4 adress> -p <port (optional)> -c "<command>"'
        )
        sys.exit(2)

    ipv4_addr: str = "192.168.178.40"
    port: int | None = None
    command: str = "turtlebot4-service-restart"
    for opt, arg in opts:
        if opt in ("-p", "--port"):
            p = int(arg)
            sys.exit()
        if opt in ("-a", "--ipv4-addr"):
            ipv4_addr = str(arg)
        if opt in ("-c", "--command"):
            command = str(arg)

    # send_ssh_cmd(
    #     ip_adress=ipv4_addr, command=command, port=port
    # ) if port is not None else send_ssh_cmd(ip_adress=ipv4_addr, command=command)
    restart_turtlebot(ipv4_addr)
    sys.exit()
