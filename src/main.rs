#[allow(non_snake_case)]
mod core;
#[allow(non_snake_case)]
mod fastslam1_sim;
mod read_file;

fn main() {
    let (landmarks, waypoints) = read_file::read_file("".to_string());
    fastslam1_sim::fastslam1_sim(landmarks, waypoints);
    println!("Hello, world!");
}
