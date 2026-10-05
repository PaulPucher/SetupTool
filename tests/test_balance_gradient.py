# Measured understeer gradient (modules/balance_gradient.py) on the real Dubai
# sample at cap 1. The reference value was established 2026-10-05 by running
# this function and the diagnostic it was lifted from (inspect_last_push.
# measure_k) on the same state: identical to machine precision.

import numpy as np

from modules.balance_gradient import estimate_understeer_gradient, balance_reading

K_DUBAI_CAP1 = 1.2660935454401083
CI_DUBAI_CAP1 = (1.1720752405094788, 1.3709700796981932)


def test_dubai_gradient_reproduces_the_diagnostic(state, effective_params, parsed_data):
    b = estimate_understeer_gradient(state, effective_params, parsed_data["laps"], parsed_data["corners"])
    assert b is not None
    assert abs(b["K_deg_per_g"] - K_DUBAI_CAP1) <= 1e-9
    assert np.allclose(b["ci95_deg_per_g"], CI_DUBAI_CAP1, atol=1e-9)
    assert b["n_samples"] == 3206 and b["n_traversals"] == 45
    assert b["accuracy_level"] == 2


def test_dubai_gradient_reads_understeer(state, effective_params, parsed_data):
    b = estimate_understeer_gradient(state, effective_params, parsed_data["laps"], parsed_data["corners"])
    lo, hi = b["ci95_deg_per_g"]
    assert lo <= b["K_deg_per_g"] <= hi
    assert balance_reading(b) == "the car understeers"


def test_reading_needs_the_range_to_exclude_zero():
    assert balance_reading({"ci95_deg_per_g": [-0.1, 0.4]}) == "balance not resolved from neutral"
    assert balance_reading({"ci95_deg_per_g": [-0.9, -0.2]}) == "the car oversteers"
