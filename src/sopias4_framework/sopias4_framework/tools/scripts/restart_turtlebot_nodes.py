"""
This scripts restarts the Turtlebot 4 nodes by connecting via SSH to the robot and running the necessary command.
"""

import getopt
import sys

import paramiko


def restart_turtlebot_nodes(
    ip_adress: str, port: int = 22, password: str = "turtlebot4"
) -> None:
    """
    Restarts the turtlebot4 nodes. For this purpose, a SSH connection is established and the necessary command is run

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

    print("Executing command...")
    # Execute command
    try:
        ssh.get_transport().open_session().exec_command(  # type: ignore
            "turtlebot4-service-restart"
        )
    except paramiko.SSHException:
        print(f"Couldn't execute the necessary ssh command via SSH on host {ip_adress}")
        return
    except Exception as e:
        print(
            f"Couldn't execute  the necessary ssh command via SSH: Error {e} on client side"
        )
    finally:
        ssh.close()

    print("Executed command! The nodes should show up in a few seconds")


if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], "p:a:", ["port", "ipv4-addr"])
    except getopt.GetoptError:
        print(
            "python3 restart_turtlebot_nodes.py -a <ipv4 adress> -p <port (optional)>"
        )
        sys.exit(2)

    ipv4_addr: str = "192.168.178.40"
    port: int | None = None
    for opt, arg in opts:
        if opt in ("-p", "--port"):
            p = int(arg)
            sys.exit()
        if opt in ("-a", "--ipv4-addr"):
            ipv4_addr = str(arg)

    restart_turtlebot_nodes(
        ipv4_addr, port
    ) if port is not None else restart_turtlebot_nodes(ipv4_addr)
    sys.exit()
