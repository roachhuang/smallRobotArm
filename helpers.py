import functools
import time


def timer(func):
    """Print the runtime of the decorated function"""
    @functools.wraps(func)
    def wrapper_timer(*args, **kwargs):
        start_time = time.perf_counter()    # 1
        value = func(*args, **kwargs)
        end_time = time.perf_counter()      # 2        
        print(f"Finished {func.__name__!r} in {end_time- start_time:.4f} secs")       
        return value
    return wrapper_timer
