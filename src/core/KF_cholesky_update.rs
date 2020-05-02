use nalgebra::{Vector2, Matrix2};

pub fn KF_cholesky_update(xf: &mut Vector2<f64>, Pf: &mut Matrix2<f64>, feat_diff: Vector2<f64>, R: Matrix2<f64>, Hf: Matrix2<f64>)
{
    let S = {
        let S = Hf*(*Pf)*Hf.transpose() + R;
        0.5 * (S + S.transpose())
    };
 
    let SChol = S.cholesky().unwrap();
    let W = *Pf * Hf.transpose() * SChol.inverse();

    *xf += W*feat_diff;
    
    *Pf -= W * W.transpose();
}