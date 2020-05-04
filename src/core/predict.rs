#[allow(unused_imports)]
use super::multivariate_gauss::multivariate_gauss;
use super::normalize_angle::normalize_angle;
use super::particle::Particle;
use nalgebra::Matrix2;
pub fn predict(particle: &mut Particle, V: f64, G: f64, _Q: Matrix2<f64>, WB: f64, dt: f64) {
    // 	if (SWITCH_PREDICT_NOISE == 1) {
    // 		Vector2d mu = {V, G};
    // 		Vector2d noise;
    // 		multivariate_gauss_base(mu,Q,noise);
    // 		V = noise[0];
    // 		G = noise[1];
    // 	}

    let alpha = particle.xv[2];
    particle.xv[0] += V * dt * (G + alpha).cos();
    particle.xv[1] += V * dt * (G + alpha).sin();
    particle.xv[2] = alpha + V * dt * G.sin() / WB;
    normalize_angle(&mut particle.xv[2]);
}
