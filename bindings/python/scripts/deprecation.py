import functools
import inspect
import warnings

class DeprecatedWarning(UserWarning):
    pass

def deprecated(instructions):
    """Flags a method as deprecated.
    Args:
        instructions: A human-friendly string of instructions, such
            as: 'Please migrate to add_proxy() ASAP.'
    """
    def decorator(func):
        '''This is a decorator which can be used to mark functions
        as deprecated. It will result in a warning being emitted
        when the function is used.'''
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            message = 'Call to deprecated function {}. {}'.format(
                func.__name__,
                instructions)

            frame = inspect.currentframe().f_back

            warnings.warn_explicit(message,
                                   category=DeprecatedWarning,
                                   filename=inspect.getfile(frame.f_code),
                                   lineno=frame.f_lineno)

            return func(*args, **kwargs)

        if wrapper.__doc__ is None:
            wrapper.__doc__ = instructions
        return wrapper

    return decorator
