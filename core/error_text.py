# One-line error text for status labels, dialogs and PDF notes. No Qt ->
# testable.

def friendly_error_text(e):
    """One readable line, never a bare repr(). ValueError messages are written
    as full sentences here -> str(e). Anything else is more likely a bug ->
    prefix the class name (a bare KeyError is just the key).
    """
    if isinstance(e, ValueError) and str(e):
        return str(e)
    return f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
