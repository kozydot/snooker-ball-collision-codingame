// https://www.codingame.com/ide/puzzle/snooker-ball-collision

use std::io;
use std::ops::{Add, Sub};

// 2d vector for physics calc
#[derive(Clone, Copy, Debug)]
struct Vec2 {
    x: f64,
    y: f64,
}

impl Vec2 {
    fn new(x: f64, y: f64) -> Self { Vec2 { x, y } }
    fn scale(&self, s: f64) -> Vec2 { Vec2::new(self.x * s, self.y * s) }
    fn dot(&self, other: &Vec2) -> f64 { self.x * other.x + self.y * other.y }
    fn mag_sq(&self) -> f64 { self.x * self.x + self.y * self.y }
    fn mag(&self) -> f64 { self.mag_sq().sqrt() }
    fn normalize(&self) -> Vec2 {
        let m = self.mag();
        if m > 1e-9 { self.scale(1.0 / m) } else { Vec2::new(0.0, 0.0) }
    }
}

impl Add for Vec2 { type Output = Self; fn add(self, other: Self) -> Self { Self { x: self.x + other.x, y: self.y + other.y } } }
impl Sub for Vec2 { type Output = Self; fn sub(self, other: Self) -> Self { Self { x: self.x - other.x, y: self.y - other.y } } }

macro_rules! parse_input { ($x:expr, $t:ident) => ($x.trim().parse::<$t>().unwrap()) }

// rounds to two decimal places (banker's) and formats output string
fn format_output(n: f64) -> String {
    const EPSILON: f64 = 1e-9;
    let scaled = n * 100.0;
    
    // perform banker's rounding
    let rounded_scaled = {
        let floor = scaled.floor();
        let diff = scaled - floor;

        // check for a halfway point (x.5)
        if (diff - 0.5).abs() < EPSILON {
            // halfway case: round to nearest even integer
            if floor % 2.0 == 0.0 { floor } else { floor + 1.0 }
        } else {
            // non-halfway case: standard rounding
            scaled.round()
        }
    };
    
    // format string to trim trailing zeros
    let scaled_int = rounded_scaled as i64;
    let val = rounded_scaled / 100.0;

    if scaled_int % 100 == 0 {
        // format as integer if no decimal part
        format!("{}", scaled_int / 100)
    } else if scaled_int % 10 == 0 {
        // format to one decimal place if applicable
        format!("{:.1}", val)
    } else {
        // format to two decimal places
        format!("{:.2}", val)
    }
}

// main entry point
fn main() {
    // physical constants
    const R: f64 = 0.03075;
    const R2: f64 = 2.0 * R;
    const R2_SQ: f64 = R2 * R2;
    const FRICTION_COEFF: f64 = 0.8;
    const DISTANCE_MULTIPLIER: f64 = 1.0 / FRICTION_COEFF; // distance = v_initial / 0.8

    // parse standard input
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    let inputs = input_line.split(" ").collect::<Vec<_>>();
    let p0 = Vec2::new(parse_input!(inputs[0], f64), parse_input!(inputs[1], f64));
    
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    let inputs = input_line.split(" ").collect::<Vec<_>>();
    let p1 = Vec2::new(parse_input!(inputs[0], f64), parse_input!(inputs[1], f64));
    
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    let inputs = input_line.split(" ").collect::<Vec<_>>();
    let v0 = Vec2::new(parse_input!(inputs[0], f64), parse_input!(inputs[1], f64));

    // core logic
    let v0_mag = v0.mag();

    // handle trivial case: no initial velocity
    if v0_mag < 1e-9 {
        println!("{} {}", format_output(p0.x), format_output(p0.y));
        println!("{} {}", format_output(p1.x), format_output(p1.y));
        return;
    }

    let v0_dir = v0.normalize();
    let dp = p0 - p1; // vector from p1 to p0
    
    // project dp onto v0_dir to find closest approach distance
    let s_closest = -dp.dot(&v0_dir);
    
    let mut will_collide = false;
    let mut s_contact = -1.0;

    // collision only possible if moving towards the target
    if s_closest >= 0.0 {
        // calculate squared minimum distance between trajectory and target center
        let dist_min_sq = dp.mag_sq() - s_closest * s_closest;
        
        // check if path is close enough for a collision
        if dist_min_sq <= R2_SQ + 1e-9 { // use epsilon for float comparison
            // calculate travel distance to the contact point
            let s_contact_candidate = s_closest - (R2_SQ - dist_min_sq).sqrt();
            
            // calculate max travel distance before friction stops the ball
            let max_dist = v0_mag * DISTANCE_MULTIPLIER;
            
            // a collision occurs if contact distance is less than max travel distance
            if s_contact_candidate >= 0.0 && s_contact_candidate <= max_dist {
                will_collide = true;
                s_contact = s_contact_candidate;
            }
        }
    }
    
    let p0_final: Vec2;
    let p1_final: Vec2;

    if !will_collide {
        // no collision scenario: ball 0 travels max distance; ball 1 is static
        let displacement = v0.scale(DISTANCE_MULTIPLIER);
        p0_final = p0 + displacement;
        p1_final = p1;
    } else {
        // collision scenario
        // calculate state at impact
        let p0_at_coll = p0 + v0_dir.scale(s_contact);
        let v0_at_coll_mag = v0_mag - FRICTION_COEFF * s_contact; // speed decreases linearly with distance
        let v0_before_coll = v0_dir.scale(v0_at_coll_mag);

        // resolve elastic collision
        let n = (p0_at_coll - p1).normalize(); // collision normal vector
        let proj = v0_before_coll.dot(&n); // project velocity onto normal
        let v1_after_coll = n.scale(proj); // ball 1 receives parallel velocity component
        let v0_after_coll = v0_before_coll - v1_after_coll; // ball 0 retains perpendicular velocity component

        // calculate final positions from post-collision travel
        let disp0 = v0_after_coll.scale(DISTANCE_MULTIPLIER);
        let disp1 = v1_after_coll.scale(DISTANCE_MULTIPLIER);
        p0_final = p0_at_coll + disp0;
        p1_final = p1 + disp1;
    }

    println!("{} {}", format_output(p0_final.x), format_output(p0_final.y));
    println!("{} {}", format_output(p1_final.x), format_output(p1_final.y));
}
