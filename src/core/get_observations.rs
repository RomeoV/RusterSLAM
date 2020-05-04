use super::normalize_angle::normalize_angle;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use nalgebra::{Vector2, Vector3};
use std::collections::{HashMap, HashSet};

fn check_if_in_sight(meas: &[f64; 2], xv: &Vector3<f64>) -> bool {
    let dx = meas[0] - xv[0];
    let dy = meas[1] - xv[1];
    return dx * xv[2].cos() + dy * xv[2].sin() >= 0.;
}

fn measurement_to_polar(meas: [f64; 2], xv: &Vector3<f64>) -> Vector2<f64> {
    let dist = ((meas[0] - xv[0]).powi(2) + (meas[1] - xv[1]).powi(2)).sqrt();
    let angle = {
        let mut angle = (meas[1] - xv[1]).atan2(meas[0] - xv[0]);
        normalize_angle(&mut angle);
        angle
    };
    return Vector2::new(dist, angle);
}

pub fn get_observations(
    xv: &Vector3<f64>,
    range_max: f64,
    landmarks: &Vec<[f64; 2]>,
    landmark_tree: &KdTree<f64, usize, &[f64; 2]>,
) -> Vec<(usize, Vector2<f64>)> {
    let xv_pos = [xv[0], xv[1]];
    return landmark_tree
        .iter_nearest(&xv_pos, &squared_euclidean)
        .unwrap()
        .map(|(_dist, idx)| (*idx, landmarks[*idx]))
        .take_while(|(_idx, p)| squared_euclidean(p, &xv_pos) <= range_max.powi(2))
        .filter(|(_idx, p)| check_if_in_sight(p, xv))
        .map(|(idx, p)| (idx, measurement_to_polar(p, xv)))
        .collect::<Vec<_>>();
}

pub fn data_associate_known<'a>(
    meas_with_idx: &'a Vec<(usize, Vector2<f64>)>,
    known_landmarks: &mut HashMap<usize, usize>,
) -> (Vec<&'a Vector2<f64>>, Vec<&'a Vector2<f64>>, Vec<usize>) {
    let mut known_indices = Vec::<usize>::new();
    let known_features: Vec<&Vector2<f64>> = meas_with_idx
        .iter()
        .filter(|(idx, _)| known_landmarks.contains_key(idx))
        .inspect(|(idx, _)| {
            known_indices.push(*idx);
        })
        .map(|(_, meas)| meas)
        .collect();

    let mut features_to_add: HashSet<usize> = HashSet::new();
    let new_features: Vec<&Vector2<f64>> = meas_with_idx
        .iter()
        .filter(|(idx, _)| !known_landmarks.contains_key(idx))
        .inspect(|(idx, _)| {
            features_to_add.insert(*idx);
        })
        .map(|(_, meas)| meas)
        .collect();

    for (i, idx) in features_to_add.iter().enumerate() {
        known_landmarks.insert(*idx, known_features.len() + i);
    }

    return (known_features, new_features, known_indices);
}
// void data_associate_known_base(cVector2d z[], const int* idz, const size_t idz_size,
//        int* table, const int Nf_known, Vector2d zf[], int *idf, size_t *count_zf, Vector2d zn[], size_t *count_zn)
//{
//    // zn and zf are always allocated but considered empty in this step
//    // idf.clear(); // dealloc or just set to zero
//    int *idn = (int*) malloc(idz_size * sizeof(int));
//
//    int i = 0, ii = 0;
//    *count_zn = 0;
//    *count_zf = 0;
//    for (i = 0; i < idz_size; i++){
//        ii = idz[i];
//        if ( table[ii] == -1 ) { // new feature
//            // zn.push_back(z[i]); // z[i] is vector2d
//            zn[*count_zn][0] = z[i][0];
//            zn[*count_zn][1] = z[i][1];
//            // idn.push_back(ii);
//            idn[*count_zn] = ii;
//            (*count_zn)++;
//        }
//        else {
//            // zf.push_back(z[i]); // z[i] is vector2d
//            zf[*count_zf][0] = z[i][0];
//            zf[*count_zf][1] = z[i][1];
//            // idf.push_back( table[ii] );
//            idf[*count_zf] = table[ii];
//            (*count_zf)++;
//        }
//    }
//
//    // assert(idn.size() == zn.size());
//    for (int i = 0; i < *count_zn; i++) {
//        table[ idn[i] ] = Nf_known + i;
//    }
//    free(idn);
//}
