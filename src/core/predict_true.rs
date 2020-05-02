use nalgebra::Vector3;
use super::normalize_angle::normalize_angle;

pub fn predict_true(mut xv: Vector3<f64>, V: f64, G: f64, WB: f64, dt: f64) -> Vector3<f64>
{
  xv[0] += V * dt * (G + xv[2]).cos();
  xv[1] += V * dt * (G + xv[2]).sin();
  xv[2] += V * dt * G.sin() / WB;
  normalize_angle(&mut xv[2]);
  return xv;
}