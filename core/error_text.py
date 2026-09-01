# Reliability pass: shared one-line error-message formatting. Pure
# Python, no Qt/PyQt6 dependency -- used by both ui/ (status labels,
# QMessageBox text) and core/ (inline PDF error notes) call sites, and
# directly testable without pulling PyQt6 into the test suite.

def friendly_error_text(e):
    """One readable line for an unexpected exception, never a bare
    Python repr().

    This codebase's own convention is to raise ValueError with a
    complete, human-readable sentence at any real validation boundary
    (e.g. the sample-rate guard's "sample rate 20 Hz, expected 50 Hz")
    -- str(e) alone is already the right text for those. Other exception
    types are more often an unanticipated bug than a deliberately-worded
    message (KeyError's str() is just the bare missing key,
    AttributeError's can be pure internals-speak) -- prefixing the class
    name at least tells the user WHAT kind of thing broke instead of
    showing an unexplained fragment with no context.
    """
    if isinstance(e, ValueError) and str(e):
        return str(e)
    return f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
