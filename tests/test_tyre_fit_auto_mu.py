# Fz-integration Phase 2: synthetic recovery test for the load-normalised
# (mu) Pacejka fit (modules.tyre_fit_auto._fit_axle_pacejka_mu). Synthetic
# fixture only -- generates noiseless Fy from a KNOWN (B, C, mu, E) with a
# varying per-sample Fz, then checks the fit recovers mu (and the other
# three parameters) back out. Not a real-data validation -- that is what
# diagnostics/inspect_fz_mu_tyre_fit.py's real-session runs are for.

import numpy as np

from modules.tyre_fit_auto import _fit_axle_pacejka_mu
from modules.tyre_model_pacejka import pacejka_lateral_force

B_TRUE = 12.0
C_TRUE = 1.9
MU_TRUE = 1.5
E_TRUE = 0.97


def _synthetic_population(n=300, fz_lo=3000.0, fz_hi=6000.0, alpha_deg_span=14.0):
    alpha = np.radians(np.linspace(-alpha_deg_span, alpha_deg_span, n))
    fz = np.linspace(fz_lo, fz_hi, n)
    fy = pacejka_lateral_force(alpha, B_TRUE, C_TRUE, MU_TRUE * fz, E_TRUE)
    base_mask = np.full(n, True)
    return alpha, fy, fz, base_mask


def test_known_mu_recovered_noiseless():
    # mu (and D=mu*mean_fz, which follows directly from it) is the
    # parameter Phase 2 actually needs recovered -- checked tightly. B/C/E
    # are NOT checked tightly: a real finding surfaced while writing this
    # test, not swept under the rug -- Powell's own joint (B, C, mu, E) fit
    # shows a genuine B/C/E interdependency even on a NOISELESS synthetic
    # population (varying the alpha range/sample density moves the
    # recovered B/C/E by 10-20% while mu stays within ~1%, and the fit's
    # own RMS residual stays small relative to D either way) -- the same
    # class of curve-parameter ambiguity already documented for this
    # project's Dugoff/Pacejka refit loops (thesis_notes.md "Refit-loop
    # conclusion: structural non-convergence..."), here appearing as a
    # within-single-fit ambiguity rather than an iterative drift. mu is
    # comparatively well identified because it scales D directly and
    # proportionally against an INDEPENDENTLY varying Fz, giving Powell a
    # signal the other three shape parameters do not have.
    alpha, fy, fz, base_mask = _synthetic_population()
    fit = _fit_axle_pacejka_mu(alpha, fy, fz, base_mask)

    assert fit["powell_converged"]
    assert fit["sign_ok"]
    assert np.isclose(fit["mu"], MU_TRUE, rtol=2e-2)
    assert np.isclose(fit["mean_axle_fz_N"], np.mean(fz))
    assert np.isclose(fit["D"], MU_TRUE * np.mean(fz), rtol=2e-2)
    # B/C/E: plausible sign/order of magnitude only, not precise recovery.
    assert fit["B"] > 0
    assert fit["C"] > 0
    assert 0.0 < fit["E"] < 2.0
    # RMS residual small relative to the peak force, not an absolute N
    # threshold -- meaningful across different D scales.
    assert fit["fit_rms_resid_N"] < 0.02 * fit["D"]


def test_mu_recovered_independent_of_fz_range_used():
    # Same B/C/mu/E, a DIFFERENT Fz range -- mu itself (a ratio, not a
    # force) must recover to the same value regardless of which load range
    # the session happened to visit; only the representative D changes.
    alpha, fy, fz, base_mask = _synthetic_population(fz_lo=5000.0, fz_hi=9000.0)
    fit = _fit_axle_pacejka_mu(alpha, fy, fz, base_mask)
    assert np.isclose(fit["mu"], MU_TRUE, rtol=2e-2)
    assert np.isclose(fit["D"], MU_TRUE * np.mean(fz), rtol=2e-2)


def test_empty_population_returns_degenerate_shape_not_a_crash():
    n = 10
    alpha = np.radians(np.linspace(-10.0, 10.0, n))
    fy = np.zeros(n)
    fz = np.full(n, 4000.0)
    base_mask = np.full(n, False)  # nothing selected -- must not reach Powell/percentile

    fit = _fit_axle_pacejka_mu(alpha, fy, fz, base_mask)
    assert fit["fit_n_samples"] == 0
    assert not fit["powell_converged"]
    assert not fit["sign_ok"]
    assert np.isnan(fit["mu"])
    assert np.isnan(fit["mean_axle_fz_N"])


def test_nan_fz_samples_excluded_from_fit_population():
    alpha, fy, fz, base_mask = _synthetic_population(n=200)
    fz = fz.copy()
    fz[:20] = np.nan  # some samples with no measured Fz this instant
    fit = _fit_axle_pacejka_mu(alpha, fy, fz, base_mask)
    assert fit["fit_n_samples"] == 180
    assert np.isclose(fit["mu"], MU_TRUE, rtol=2e-2)
