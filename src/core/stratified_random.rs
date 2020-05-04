use rand::{thread_rng, Rng};

pub fn stratified_random(N: usize) -> Vec<f64> {
    let k: f64 = 1.0 / N as f64;
    let mut result = Vec::with_capacity(N);
    //deterministic intervals

    let mut rng = thread_rng();
    for i in 0..N {
        result.push(k * (i as f64) + k * rng.gen::<f64>());
    }

    return result;
}

pub fn normalize_weights(weights: &mut Vec<f64>) {
    let w_sum: f64 = weights.iter().sum();
    for w in weights.iter_mut() {
        *w /= w_sum;
    }
}

pub fn stratified_resample(mut weights: Vec<f64>) -> (f64, Vec<usize>) {
    normalize_weights(&mut weights);
    let weights = weights; // make constant
    let w_squared_sum: f64 = weights.iter().map(|w| w * w).sum();

    let select = stratified_random(weights.len());
    let weights_cumsum: Vec<f64> = weights
        .iter()
        .scan(0., |state, x| {
            *state += *x;
            Some(*state)
        })
        .collect();

    let mut keep: Vec<usize> = Vec::with_capacity(weights.len());
    let mut ctr: usize = 0;
    for i in 0..weights.len() {
        while ctr < weights.len() && select[ctr] < weights_cumsum[i] {
            keep.push(i);
            ctr += 1;
        }
    }

    return (1. / w_squared_sum, keep);
}
