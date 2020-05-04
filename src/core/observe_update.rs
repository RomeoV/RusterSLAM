use super::add_feature::add_feature;
use super::compute_weight::compute_weight;
use super::feature_update::feature_update;
use super::get_observations::{data_associate_known, get_observations};
use super::particle::Particle;
use super::resample_particles::resample_particles;
use kdtree::KdTree;
use nalgebra::{Matrix2, Vector3};
use std::collections::HashMap;

pub fn observe_update<'a, 'b>(
    xv: &Vector3<f64>,
    range_max: f64,
    particles: &'a mut Vec<Particle>,
    waypoints: &Vec<[f64; 2]>,
    waypoint_tree: &KdTree<f64, usize, &[f64; 2]>,
    known_features_map: &mut HashMap<usize, usize>,
    R: &Matrix2<f64>,
) {
    let observations_with_idx = get_observations(&xv, range_max, &waypoints, &waypoint_tree);

    let (known_features, new_features, idf) =
        data_associate_known(&observations_with_idx, known_features_map);

    for p in particles.iter_mut() {
        if known_features.len() > 0 {
            let w = compute_weight(p, &known_features, &idf, &R);
            p.weight *= w;
            feature_update(p, &known_features, &idf, &R);
        }
        if new_features.len() > 0 {
            add_feature(p, &new_features, &R);
        }
    }

    resample_particles(particles);
}
