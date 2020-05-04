use super::normalize_angle::normalize_angle;
use super::particle::Particle;
use nalgebra::{Matrix2, Matrix2x3, Vector2};

#[derive(Default)]
pub struct JacobianResults {
    pub z_pred: Vec<Vector2<f64>>,
    pub Hv: Vec<Matrix2x3<f64>>,
    pub Hf: Vec<Matrix2<f64>>,
    pub Sf: Vec<Matrix2<f64>>,
}

#[allow(unused_variables)]
pub fn compute_jacobians(
    particle: &Particle,
    idf: &Vec<usize>,
    R: &Matrix2<f64>,
    jac_results: &mut JacobianResults,
) {
    jac_results.z_pred.clear();
    jac_results.Hv.clear();
    jac_results.Hf.clear();
    jac_results.Sf.clear();

    let xv = &particle.xv;
    for i in idf.iter() {
        let xf_ = particle.xf[*i];
        let Pf_ = particle.Pf[*i];

        let dx = xf_[0] - xv[0];
        let dy = xf_[1] - xv[1];
        let d2 = dx.powi(2) + dy.powi(2);
        let d = d2.sqrt();
        let angle = {
            let mut angle = dy.atan2(dx) - xv[2];
            normalize_angle(&mut angle);
            angle
        };

        jac_results.z_pred.push(Vector2::new(d, angle));

        let Hv = Matrix2x3::new(-dx / d, -dy / d, 0., dy / d2, -dx / d2, -1.);
        jac_results.Hv.push(Hv);

        let Hf = Matrix2::new(dx / d, dy / d, -dy / d2, dx / d2);
        jac_results.Hf.push(Hf);

        jac_results.Sf.push(Hf * Pf_ * Hf.transpose() + R);
    }
    *jac_results = Default::default();
}
