use super::compute_jacobians::{compute_jacobians, JacobianResults};
use super::normalize_angle::normalize_angle;
use super::particle::Particle;
use itertools::izip;
use nalgebra::{Matrix2, Vector2};

pub fn compute_weight(
    particle: &Particle,
    zs: &Vec<&Vector2<f64>>,
    idf: &Vec<usize>,
    R: &Matrix2<f64>,
) -> f64 {
    let M_PI = std::f64::consts::PI;

    let mut jacobian_results: JacobianResults = Default::default();
    compute_jacobians(&particle, &idf, &R, &mut jacobian_results);
    let JacobianResults {
        z_pred,
        Hv: _Hv,
        Hf: _Hf,
        Sf,
    } = jacobian_results;

    let mut w = 1.;
    for (z_meas, z_pred, Sf_) in izip!(zs, &z_pred, Sf) {
        let z_diff = {
            let mut z_diff = *z_meas - z_pred;
            normalize_angle(&mut z_diff[1]);
            z_diff
        };

        let Sf_inv = Sf_.cholesky().unwrap().inverse();
        let num = (-0.5 * z_diff.transpose() * Sf_inv * z_diff)[(0, 0)].exp();
        let den = (2. * M_PI * Sf_.determinant()).sqrt();
        w *= num / den;
    }

    return w;
}
