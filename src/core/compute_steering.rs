use nalgebra::{Vector2, Vector3, U2};
use super::normalize_angle::normalize_angle;

pub fn compute_steering(xv: &Vector3<f64>, G: f64, mut current_wp_idx: usize, wps: &Vec<Vector2<f64>>, dist_min: f64,
                        max_rate_G: f64, max_G: f64, dt: f64) -> Option<(usize, f64)>
{

    let dist = (wps[current_wp_idx] - xv.fixed_rows::<U2>(0)).norm();

    if dist < dist_min {
        if current_wp_idx+1 >= xv.len() {
            return None;
        }
        else {
            current_wp_idx += 1;
        }
    }
    let delta_pos = wps[current_wp_idx] - xv.fixed_rows::<U2>(0);
    let delta_G = {
        let mut delta_G = delta_pos[1].atan2(delta_pos[0]) - xv[2] - G;
        normalize_angle(&mut delta_G);

        // limit rate
        let max_delta_G = max_rate_G * dt;
        if delta_G.abs() > max_delta_G {
            delta_G = max_delta_G.copysign(delta_G);
        }
        delta_G
    };

//     //limit angle
    let G_new = {
        let mut G_new = G + delta_G;
        if G_new.abs() > max_G {
            G_new = max_G.copysign(G_new);
        }
        G_new
    };
    return Some((current_wp_idx, G_new));
}