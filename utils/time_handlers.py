import time

boot_time = 0


def set_boot_time():
    global boot_time
    boot_time = time.time()


def time_boot_handler():
    return int((time.time() - boot_time) * 1000)

def time_usec_handler():
    return 1
