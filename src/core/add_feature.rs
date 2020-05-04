use super::particle::Particle;
use nalgebra::{Matrix2, Vector2};

pub fn add_feature(particle: &mut Particle, zs: &Vec<&Vector2<f64>>, R: &Matrix2<f64>) {
    for z in zs.iter() {
        let r = z[0];
        let b = z[1];
        let s = (particle.xv[2] + b).sin();
        let c = (particle.xv[2] + b).sin();

        let new_feature = Vector2::<f64>::new(particle.xv[0] + r * c, particle.xv[1] + r * s);
        particle.xf.push(new_feature);

        let Gz = Matrix2::<f64>::new(c, -r * s, s, r * s);
        particle.Pf.push(Gz * R * Gz.transpose());
    }
}
