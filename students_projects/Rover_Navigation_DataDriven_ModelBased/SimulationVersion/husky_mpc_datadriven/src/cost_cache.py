"""
CostCache: A class implementing a cache for cost-related data and target values.
"""

class CostCache:
    _instance = None  

    """
    Creates a new instance of CostCache using the Singleton pattern.
    
    :param cls: The class being instantiated.
    :param *args: Variable length argument list.
    :param **kwargs: Arbitrary keyword arguments.
    :return: The existing instance if available, otherwise a new instance.
    """
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(CostCache, cls).__new__(cls, *args, **kwargs)
            cls._instance.init_cache()
        return cls._instance

    """
    Initializes the cache attributes.
    """
    def init_cache(self):
        self.T = None
        self.target_x = []
        self.target_y = []
    
    """
    Retrieves the cached value T.
    
    :return: The cached value T.
    """
    def get_T(self):
        return self.T

    """
    Sets the cached value T to a new value.
    
    :param T: The new value to be cached.
    """
    def set_T(self, T):
        self.T = T

    """
    Sets the next target's x and y values.
    
    :param x: The x value of the next target.
    :param y: The y value of the next target.
    """
    def set_next_target(self, x, y):
        self.target_x = x
        self.target_y = y

    """
    Retrieves the x and y values of the target at the specified index.
    
    :param index: The index of the target values to retrieve.
    :return: A list containing the x and y values of the target.
    """
    def next_target(self, index):
        return [self.target_x[index], self.target_y[index]]
