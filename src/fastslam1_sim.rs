use nalgebra::{Vector3, Matrix2, Matrix3};
use kdtree::KdTree;
use std::collections::HashMap;
use crate::core::{particle::Particle,
                  predict_update::predict_update,
                  observe_update::observe_update};

pub fn fastslam1_sim(
    landmarks: Vec<[f64; 2]>,
    waypoints: Vec<[f64; 2]>
)
{
    const NPARTICLES: usize = 500;
    const MAX_FEATURES: usize = 100;
    const RANGE_MAX: f64 = 30.;
    let R: Matrix2<f64> = Matrix2::identity();

    let mut particles: Vec<Particle> = Vec::with_capacity(NPARTICLES);

    let mut known_features_map: HashMap<usize, usize> = HashMap::new();

    for p in particles.iter_mut() {
        *p = Particle {
            weight: 1./NPARTICLES as f64,
            xv: Vector3::new(0., 0., 0.),
            Pv: Matrix3::zeros(),
            Nf: 0,
            xf: Vec::with_capacity(MAX_FEATURES),
            Pf: Vec::with_capacity(MAX_FEATURES)
        };
    }

    let mut landmark_tree = KdTree::new(2);

    for (i, f) in landmarks.iter().enumerate() {
        landmark_tree.add(f, i).unwrap();
    }

    const DT: f64 = 0.025;
    let mut dtsum = 0.;
    let mut T = 0.;
    let mut maybe_wp: Option<usize> = Some(0);
    let V = 1.;
    let mut G = 0.;

    let mut x_true = Vector3::new(0., 0., 0.);

    while let Some(iwp) = maybe_wp {
        if T > 1000. { break; }

        maybe_wp = predict_update(
            &mut x_true,
            &mut particles,
            V,
            &mut G,
            iwp,
            &waypoints,
            DT
        );

        dtsum += DT;
        T += DT;
        if dtsum >= 8.*DT {
            dtsum = 0.;
            observe_update(
                &x_true,
                RANGE_MAX,
                &mut particles,
                &landmarks,
                &landmark_tree,
                &mut known_features_map,
                &R
            );
        }
    }
}
//         if ( dtsum >= DT_OBSERVE ) {
//             dtsum = 0;
//             
//             //////////////////////////////////////////////////////////////
//             // Observation
//             //////////////////////////////////////////////////////////////
// 
//             observe_update(lm, N_features, xtrue, *R, ftag, 
//             da_table, ftag_visible, z, &Nf_visible, zf, idf, 
//             zn, particles, weights);
// 
//             //////////////////////////////////////////////////////////////
//         }
//     double dt        = DT_CONTROLS; // change in time btw predicts
//     double dtsum     = 0;           // change in time since last observation
//     double T         = 0;
//     int iwp          = 0;           // index to first waypoint
//     double G         = 0;           // initialize steering angle
//     double V         = V_; 
//     size_t Nf_visible = 0;
// 
//     // Main loop
//     while ( iwp != -1 ) {
// 
//         //////////////////////////////////////////////////////////////////
//         // Prediction
//         //////////////////////////////////////////////////////////////////
// 
//         predict_update(wp, N_waypoints, V, *Q, dt, NPARTICLES, xtrue, &iwp, &G,particles);
// 
//         /////////////////////////////////////////////////////////////////
// 
// 
//         //Update time
//         dtsum = dtsum + dt;
//         T+=dt;
// 
//         // Observation condition
//         if ( dtsum >= DT_OBSERVE ) {
//             dtsum = 0;
//             
//             //////////////////////////////////////////////////////////////
//             // Observation
//             //////////////////////////////////////////////////////////////
// 
//             observe_update(lm, N_features, xtrue, *R, ftag, 
//             da_table, ftag_visible, z, &Nf_visible, zf, idf, 
//             zn, particles, weights);
// 
//             //////////////////////////////////////////////////////////////
//         }
//     }

// void fastslam1_sim( double* lm, const size_t lm_rows, const size_t lm_cols, 
//                     double* wp, const size_t wp_rows, const size_t wp_cols, 
//                     Particle **particles_, double** weights_) 
// {
//     const size_t N_features = lm_rows;
//     const size_t N_waypoints = wp_rows;
// 
//     Particle *particles;
//     double *weights;
//     Vector3d xtrue   = {0,0,0};
//     setup_initial_particles(&particles, &weights, N_features, xtrue);
//     setup_initial_Q_R();  // modifies global variables
// 
//     int *ftag;
//     int *da_table;
//     setup_landmarks(&ftag, &da_table, N_features);
// 
//     Vector2d *z;  // This is a dynamic array of Vector2d - see https://stackoverflow.com/a/13597383/5616591
//     Vector2d *zf;
//     Vector2d *zn;
//     int *idf, *ftag_visible;
//     setup_measurements(&z, &zf, &zn, &idf, &ftag_visible, N_features);
// 
// //    if ( SWITCH_PREDICT_NOISE ) {
// //        printf("Sampling from predict noise usually OFF for FastSLAM 2.0\n");	
// //    }
//  
//     if ( SWITCH_SEED_RANDOM ) {
//         srand( SWITCH_SEED_RANDOM );
//     }	
// 
//     double dt        = DT_CONTROLS; // change in time btw predicts
//     double dtsum     = 0;           // change in time since last observation
//     double T         = 0;
//     int iwp          = 0;           // index to first waypoint
//     double G         = 0;           // initialize steering angle
//     double V         = V_; 
//     size_t Nf_visible = 0;
// 
//     // Main loop
//     while ( iwp != -1 ) {
// 
//         //////////////////////////////////////////////////////////////////
//         // Prediction
//         //////////////////////////////////////////////////////////////////
// 
//         predict_update(wp, N_waypoints, V, *Q, dt, NPARTICLES, xtrue, &iwp, &G,particles);
// 
//         /////////////////////////////////////////////////////////////////
// 
// 
//         //Update time
//         dtsum = dtsum + dt;
//         T+=dt;
// 
//         // Observation condition
//         if ( dtsum >= DT_OBSERVE ) {
//             dtsum = 0;
//             
//             //////////////////////////////////////////////////////////////
//             // Observation
//             //////////////////////////////////////////////////////////////
// 
//             observe_update(lm, N_features, xtrue, *R, ftag, 
//             da_table, ftag_visible, z, &Nf_visible, zf, idf, 
//             zn, particles, weights);
// 
//             //////////////////////////////////////////////////////////////
//         }
//     }
// 
//     cleanup_landmarks(&ftag, &da_table);
//     cleanup_measurements(&z, &zf, &zn, &idf, &ftag_visible);
//     *particles_ = particles;
//     *weights_ = weights;
// }