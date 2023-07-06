from multiprocessing import Pipe, Process
from multiprocessing.connection import Connection


def elevate_func_to_main_thread(func):
    """
    Elevates a function to be executed in a main thread by running it inside a Process and utilizing inter-process-communication
    """
    # Create pipe for communication
    conn1, conn2 = Pipe()
    # Run function in process with sender connection passed in
    print("start process")
    proccess = Process(
        target=__execute_in_process,
        args=(
            func,
            conn2,
        ),
    )
    proccess.start()

    return_value = conn1.recv()
    proccess.kill()

    return return_value


def __execute_in_process(func, connection: Connection):
    """
    Executes a function and return its value  with help of a pipe connection
    """
    print("Execute func")
    return_value = func()
    print(("Got value"))
    connection.send(return_value)
