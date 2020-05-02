pub fn normalize_angle(ang: &mut f64)
{
    let M_PI = std::f64::consts::PI;
    if (*ang <= (-2.* M_PI)) || (*ang > (2.*M_PI)) {
        let n = (*ang/(2.*M_PI)).floor();
        *ang = *ang-n*(2.*M_PI);    
    }
    if *ang > M_PI {
        *ang = *ang - (2.*M_PI);
    }
    if *ang <= -M_PI {
        *ang = *ang + (2.*M_PI);
    }
}
