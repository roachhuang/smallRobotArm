from functools import wraps
import time

def timer(func):
    """Print the runtime of the decorated function"""
    @wraps(func)
    def wrapped(*args, **kwargs):
        start_time = time.perf_counter()    # 1
        res = func(*args, **kwargs)
        end_time = time.perf_counter()      # 2        
        print(f"Finished {func.__name__!r} in {end_time- start_time:.4f} secs")       
        return res
    return wrapped
