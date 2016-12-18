def is_iterable(maybe_iter, unless=(basestring, dict)):
    """
        Checks if maybe_iter is is_iterable if not in unless.
    """

    try:
        iter(maybe_iter)
    except TypeError:
        return False
    return not isinstance(maybe_iter, unless)

def iterate(maybe_iter, unless=(basestring, dict)):
    """
        Returns maybe_iter as [maybe_iter] if the instance isn't iteratable and
        not in unless.
    """
    if is_iterable(maybe_iter, unless=unless):
        return maybe_iter
    return [maybe_iter]
