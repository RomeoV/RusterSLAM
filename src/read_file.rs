pub fn read_file(_s: String) -> (Vec<[f64;2]>, Vec<[f64;2]>) {
let landmarks: Vec<[f64;2]> = vec![
    [2.9922, -25.7009],
    [32.8988, -33.1776],
    [24.7991, -68.3801],
    [75.2664, -65.5763],
    [73.7087, -35.6698],
    [98.6308,   3.8941],
    [49.4097,  26.9470],
    [80.2508,  59.9688],
    [54.7056,  88.6293],
    [13.2726,  80.8411],
    [-16.3224,  49.0654],
    [-65.8551,  83.6449],
    [-96.3847,  60.5919],
    [-76.1355,  36.2928],
    [-87.3505, -21.3396],
    [-103.5498, -32.2430],
    [-92.9579, -77.7259],
    [-55.8863, -55.9190],
    [-35.3255, -19.1589],
    [-56.1978,  16.3551],
    [-7.2882,  19.7819],
    [25.7336,   3.2710],
    [-19.5610,  80.1444],
    [-41.7506,  46.2325],
    [25.4021,  26.8543],
    [91.3870,  28.2385],
    [-13.7216, -79.0338],
    [-52.8454, -92.1833],
    [-86.1298,  15.0890],
    [-127.0053,  22.7018],
    [-25.9843,   4.7078],
    [56.3508, -17.4388],
    [51.6793, -83.1863],
    [21.3146, -96.3358],
    [48.1757,  57.9979]];

let waypoints: Vec<[f64;2]> = vec![
    [12.6495, -41.5888],
    [44.7368, -54.9844],
    [85.5467, -45.0156],
    [93.6464, -17.2897],
    [64.9860,   5.7632],
    [71.8396,  31.6199],
    [71.2165,  70.5607],
    [33.5218,  76.4798],
    [12.9611,  51.5576],
    [-32.5218,  67.4455],
    [-74.8894,  69.0031],
    [-97.3193,  41.9003],
    [-107.5997,   6.3863],
    [-86.4159, -25.7009],
    [-83.9237, -64.0187],
    [-39.3754, -81.4642],
    [-17.5685, -51.5576]];
  
    return (landmarks, waypoints);
}
// use std::fs::File;
// use std::io::{self, BufRead};
// use std::path::Path;
// use regex::Regex;
// 
// fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
// where P: AsRef<Path>, {
//     let file = File::open(filename)?;
//     Ok(io::BufReader::new(file).lines())
// }
// 
// fn read_file(s: String) -> (Vec<[f64;2]>, Vec<[f64;2]>) {
//     let mut lines = read_lines(s).unwrap();
//     lines.next();
//     let lm_line = lines.next().unwrap().unwrap();
//     let row_col_regex = Regex::new(r"lm ([0-9]+) ([0-9]+)").unwrap();
//     let caps = row_col_regex.captures(&*lm_line).unwrap();
//     let (row, _col) = (caps.get(1).unwrap().as_str(), caps.get(2).unwrap().as_str());
// 
//     let landmarks: Vec<[f64;2]> = Vec::with_capacity(row.parse().unwrap());
//     let position_regex = Regex::new(r"([0-9]+) ([0-9]+)").unwrap();
//     return (Default::default(),
//             Default::default());
// }