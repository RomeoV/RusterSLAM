use nalgebra::{Vector2, Matrix2};
use rand::{Rng, thread_rng};
use rand::distributions::Uniform;

pub fn multivariate_gauss(mu: Vector2<f64>, Sigma: Matrix2<f64>) -> Vector2<f64>
{
    let L = Sigma.cholesky().unwrap().l();

    let rng = thread_rng();
    let X = Vector2::from_iterator(rng.sample_iter(Uniform::new(-1.,1.)));
    
    return mu + L * X;
}