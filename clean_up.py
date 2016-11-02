import os
import signal


def clean_up(active_processes, opened_files):
    _terminate_all_processes(active_processes)
    _close_all_opened_files(opened_files)


def _terminate_all_processes(processes):
    """
    Terminates all processes.
    :param processes: the list of active processes
    :type processes: list
    """
    for p in processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except KeyboardInterrupt:
            pass
    print('cleaned up')


def _close_all_opened_files(opened_files):
    for f in opened_files:
        f.flush()
        f.close()
