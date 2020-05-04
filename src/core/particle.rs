use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use Vec;

#[allow(non_snake_case)]
#[derive(Clone)]
pub struct Particle {
    pub weight: f64,
    pub xv: Vector3<f64>,
    pub Pv: Matrix3<f64>,
    pub Nf: u32,
    pub xf: Vec<Vector2<f64>>,
    pub Pf: Vec<Matrix2<f64>>,
}
