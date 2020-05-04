use super::particle::Particle;
use super::stratified_random::{stratified_resample};

pub fn resample_particles<'a, 'b: 'a>(particles: &'a mut Vec<Particle>) {
    // What is happening here:
    // Basically, we want to prevent deleting and reallocating memory,
    // since the total memory requirements after resampling are the same as before.
    // In order to resample in-place (i.e. without new allocations), we have to be careful
    // though to not override a particle with a new value while it's contents are still
    // needed for copying later.
    //
    // Idea:
    // We basically create a DAG of which particles depend on which other particles,
    // i.e. will copy from the other particle.
    // Then we check the DAG for particles without dependencies (i.e. which will not be copied from),
    // process/fill it and remove it from the DAG, thus loosing the information of that particle.
    // We repeat this until all particles have been copied.
    //
    // This idea relies on the fact that `keep_indices` is sorted and thus will not create
    // any circular dependencies.
    //
    // Note that particles that kopy itself (i.e. `keep_indices[i] == i`) will stay in the
    // DAG forever and will thus not be copied --- but luckily they already contain exactly
    // what they should contain!
    let weights: Vec<f64> = particles.iter().map(|p| p.weight).collect();
    let (N_eff, keep_indices) = stratified_resample(weights);

    let N_min: f64 = 0.75 * particles.len() as f64;
    if N_eff < N_min {
        let mut num_occurences = count_occurences(&keep_indices);
        while let Some(i) = find_particle_without_dependency(&num_occurences) {
            num_occurences[i] = 100; // just a num > 0
            particles[keep_indices[i]] = particles[i].clone();
        }
        let NPARTICLES = particles.len();
        for p in particles.into_iter() {
            p.weight = 1./NPARTICLES as f64;
        }
    }
}

fn find_particle_without_dependency(count: &Vec<usize>) -> Option<usize> {
    for i in 0..count.len() {
        if count[i] == 0 {
            return Some(i);
        }
    }
    return None;
}

fn count_occurences(keep_indices: &Vec<usize>) -> Vec<usize> {
    let mut num_occurences: Vec<usize> = Vec::with_capacity(keep_indices.len());

    for i in 0..keep_indices.len() {
        num_occurences[keep_indices[i]] += 1;
    }

    return num_occurences;
}
