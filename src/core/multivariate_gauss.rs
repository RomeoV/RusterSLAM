use nalgebra::{Matrix2, Vector2};
use rand::distributions::Uniform;
use rand::{thread_rng, Rng};

#[allow(dead_code)]
pub fn multivariate_gauss(mu: Vector2<f64>, Sigma: Matrix2<f64>) -> Vector2<f64> {
    let L = Sigma.cholesky().unwrap().l();

    let rng = thread_rng();
    let X = Vector2::from_iterator(rng.sample_iter(Uniform::new(-1., 1.)));

    return mu + L * X;
}
