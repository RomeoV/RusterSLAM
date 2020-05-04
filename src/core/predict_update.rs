use super::compute_steering::compute_steering;
use super::particle::Particle;
use super::predict::predict;
use super::predict_true::predict_true;
use nalgebra::{Matrix2, Vector3};

pub fn predict_update(
    x_true: &mut Vector3<f64>,
    particles: &mut Vec<Particle>,
    V: f64,
    G: &mut f64,
    current_wp_idx: usize,
    wps: &Vec<[f64; 2]>,
    dt: f64,
) -> Option<usize> {
    let M_PI = std::f64::consts::PI;
    let dist_min = 1.;
    let max_G = 30.*M_PI/180.;
    let max_rate_G = 20.*M_PI/180.;
    let WB = 4.;
    match compute_steering(
        x_true,
        *G,
        current_wp_idx,
        wps,
        dist_min,
        max_rate_G,
        max_G,
        dt,
    ) {
        Some((iwp, G_new)) => {
            *x_true = predict_true(*x_true, V, G_new, WB, dt);

            for p in particles.iter_mut() {
                *G = G_new;
                predict(p, V, *G, Matrix2::zeros(), WB, dt);
            }
            return Some(iwp);
        }
        None => return None,
    }
}
// void predict_update_base(double* wp, size_t N_waypoints, double V, double* Q, double dt,
//                     size_t N, Vector3d xtrue, int* iwp, double* G, Particle* particles) {
//     compute_steering_base(xtrue, wp, N_waypoints, AT_WAYPOINT, RATEG, MAXG, dt, iwp, G);
//             if ( *iwp == -1 && NUMBER_LOOPS > 1 ) {
//                 *iwp = 0;
//                 NUMBER_LOOPS--;
//             }
//     predict_true_base(V, *G, WHEELBASE, dt, xtrue);
//
//     // add process noise
//     double VnGn[2];
//     add_control_noise_base(V, *G, Q, SWITCH_CONTROL_NOISE, VnGn); // TODO
//     // Predict step
//     for (size_t i = 0; i < NPARTICLES; i++) {
//         predict_base(&particles[i], VnGn[0], VnGn[1], Q, WHEELBASE, dt);
//     }
// }
