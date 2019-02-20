import functools
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

            warnings.warn(message, category=DeprecatedWarning, stacklevel=2)

            return func(*args, **kwargs)

        instructions_doc = 'Deprecated: ' + instructions
        if wrapper.__doc__ is None:
            wrapper.__doc__ = instructions_doc
        else:
            wrapper.__doc__ = wrapper.__doc__.rstrip() + '\n\n' + instructions_doc
        return wrapper

    return decorator
