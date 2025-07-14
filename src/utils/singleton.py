def singleton(cls):
    """
    A decorator to make a class a singleton.
    """
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            # If an instance does not already exist, create one and store it
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance
