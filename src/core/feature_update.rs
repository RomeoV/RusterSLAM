use super::normalize_angle::normalize_angle;
use super::particle::Particle;
use super::compute_jacobians::{compute_jacobians, JacobianResults};
use super::KF_cholesky_update::KF_cholesky_update;
use nalgebra::{Vector2, Matrix2};
use itertools::izip;


fn feature_update(particle: &mut Particle,
                  zs: &Vec<Vector2<f64>>,
                  idf: &Vec<usize>,
                  R: &Matrix2<f64>) {
  // Having selected a new pose from the proposal distribution, this pose is
  // assumed perfect and each feature update maybe computed independently and
  // without pose uncertainty

  let mut jacobian_results: JacobianResults = Default::default();
  compute_jacobians(&particle, &idf, &R, &mut jacobian_results);
  let JacobianResults{z_pred, Hv: _Hv, Hf, Sf: _Sf} = jacobian_results;

  for (z_meas, z_pred, Hf_, i_) in izip!(zs, &z_pred, Hf, idf) {
      let mut xf_ = &mut particle.xf[*i_];
      let mut Pf_ = &mut particle.Pf[*i_];

      let z_diff = {
        let mut z_diff = z_meas - z_pred;
        normalize_angle(&mut z_diff[1]);
        z_diff
      };


      KF_cholesky_update(&mut xf_, &mut Pf_, z_diff, *R, Hf_);
  }
}