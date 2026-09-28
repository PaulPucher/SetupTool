# Manual override of a classification threshold (parameters.json
# classification.<key>), recorded config-side. The data-derived value is
# kept once, machine-readable, as derived_default: written on the first
# override, never overwritten by later edits, removed on restore. The
# derivation rule (re-derive from this car's own distribution) applies to
# that default; an override never rewrites it. derived_from stays the
# human-facing, append-only record.

_TOL = 1e-9


def is_manual(entry):
    return "derived_default" in entry


def apply_threshold_edit(entry, new_value, date_str):
    """Mutates entry. Returns "unchanged", "manual" or "restored".
    Setting the value back to derived_default is a restore, however the
    user got there (restore control or typing it in)."""
    old_value = float(entry["value"])
    new_value = float(new_value)
    if abs(new_value - old_value) <= _TOL:
        return "unchanged"

    if is_manual(entry) and abs(new_value - float(entry["derived_default"])) <= _TOL:
        entry["value"] = entry.pop("derived_default")
        entry["derived_from"] = (entry.get("derived_from", "")
                                 + f"; restored to data-derived default {date_str}")
        return "restored"

    if not is_manual(entry):
        entry["derived_default"] = old_value
    entry["value"] = new_value
    entry["derived_from"] = (entry.get("derived_from", "")
                             + f"; manually set {date_str}; data-derived default: "
                               f"{entry['derived_default']}")
    return "manual"
