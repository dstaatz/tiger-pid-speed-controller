/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


use std::time::Instant;
use std::cmp::Ordering;

use pid::Pid;
use rosrust_msg::std_msgs::Float64;
use rosrust_msg::geometry_msgs::Pose2D;


#[derive(Debug, Clone, PartialEq)]
pub struct PidConstants {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub p_limit: f64,
    pub i_limit: f64,
    pub d_limit: f64,
}

#[derive(Debug, Clone)]
pub struct SpeedPidController {
    pid: Pid<f64>,
    prev_pose: Option<(Instant, Pose2D)>,
    constant_speed: f64,
}

impl SpeedPidController {

    /// Create a new controller
    pub fn new(c: PidConstants, constant_speed: f64) -> Self {
        let pid = Pid::new(c.kp, c.ki, c.kd, c.p_limit, c.i_limit, c.d_limit, 0.0);
        Self { 
            pid,
            prev_pose: None,
            constant_speed
        }
    }

    // Update the setpoint
    pub fn update_setpoint(&mut self, input: Float64) {

        match input.data.partial_cmp(&0.0) {
            Some(Ordering::Greater) => self.pid.setpoint = self.constant_speed,
            Some(Ordering::Less) => self.pid.setpoint = -1.0 * self.constant_speed,
            Some(Ordering::Equal) => self.pid.setpoint = 0.0,
            None => self.pid.setpoint = 0.0,
        }
    }

    /// Determine the Steering output based on new measurement
    pub fn update_pose(&mut self, pose: Pose2D) -> Float64 {

        match &self.prev_pose {
            None => {
                self.prev_pose = Some((Instant::now(), pose));
                Float64 { data: 0.0 }
            },
            Some(prev) => {

                let speed = calc_speed_from_pose(prev, &pose);
                let output = self.pid.next_control_output(speed);

                rosrust::ros_info!("(setpoint speed, output): ({}, {}, {})", self.pid.setpoint, speed, output.output);

                self.prev_pose = Some((Instant::now(), pose));

                Float64 { data: output.output }
            },
        }
    }
}


// Determine the magnitude of the speed in 2d space based on two stamped locations
fn calc_speed_from_pose(prev: &(Instant, Pose2D), cur: &Pose2D) -> f64 {
    
    let delta_time = prev.0.elapsed().as_secs_f64();

    let delta_x = cur.x - prev.1.x;
    let delta_y = cur.y - prev.1.y;
    
    let t_theta = f64::atan2(delta_y, delta_x); // Theta formed by delta's (translational theta)
    let r_theta = cur.theta; // robot's global theta

    // Direction is inverted because x-axis points backwards on our car
    let forward = !direction(t_theta, r_theta);
    let direction = if forward { 1.0 } else { -1.0 };

    direction * (delta_x*delta_x + delta_y*delta_y).sqrt() / delta_time
}


/// Determine wether this step had forward velocity or backwards velocity using car model
/// Basically we want to know if r_theta is between in the range t_theta +- FRAC_PI_2
/// but it is made more complicated by overflows and underflows in the range +-PI
fn direction(t_theta: f64, r_theta: f64) -> bool {
    
    use std::f64::consts::{PI, FRAC_PI_2};
    const TWO_PI: f64 = 2.0 * PI;
    
    if t_theta <= PI && t_theta >= -PI {
        if t_theta <= FRAC_PI_2 && t_theta >= -FRAC_PI_2 {
            (t_theta - FRAC_PI_2) <= r_theta && r_theta <= (t_theta + FRAC_PI_2)
        } else if t_theta <= FRAC_PI_2 {
            if r_theta > 0.0 {
                r_theta >= (t_theta - FRAC_PI_2 + TWO_PI)
            } else {
                r_theta <= (t_theta + FRAC_PI_2)
            }
        } else {
            if r_theta > 0.0 {
                r_theta >= (t_theta - FRAC_PI_2)
            } else {
                r_theta <= (t_theta + FRAC_PI_2 - TWO_PI)
            }
        }
    } else {
        // Error kinda
        true
    }
}


#[cfg(test)]
mod tests {

    use std::f64::consts::{PI, FRAC_PI_2, FRAC_PI_4, FRAC_PI_6};
    
    use super::*;

    #[test]
    fn test_direction() {

        // Test circle when t_theta = 0.0
        assert_eq!(direction(0.0, PI),             false);
        assert_eq!(direction(0.0, 5.0*FRAC_PI_6),  false);
        assert_eq!(direction(0.0, 3.0*FRAC_PI_4),  false);
        assert_eq!(direction(0.0, 4.0*FRAC_PI_6),  false);
        assert_eq!(direction(0.0, FRAC_PI_2),      true);
        assert_eq!(direction(0.0, 2.0*FRAC_PI_6),  true);
        assert_eq!(direction(0.0, FRAC_PI_4),      true);
        assert_eq!(direction(0.0, FRAC_PI_6),      true);
        assert_eq!(direction(0.0, 0.0),            true);
        assert_eq!(direction(0.0, -FRAC_PI_6),     true);
        assert_eq!(direction(0.0, -FRAC_PI_4),     true);
        assert_eq!(direction(0.0, -2.0*FRAC_PI_6), true);
        assert_eq!(direction(0.0, -FRAC_PI_2),     true);
        assert_eq!(direction(0.0, -4.0*FRAC_PI_6), false);
        assert_eq!(direction(0.0, -3.0*FRAC_PI_4), false);
        assert_eq!(direction(0.0, -5.0*FRAC_PI_6), false);
        assert_eq!(direction(0.0, -PI),            false);

        // Test circle when t_theta = 3.0*FRAC_PI_4
        assert_eq!(direction(3.0*FRAC_PI_4, PI),             true);
        assert_eq!(direction(3.0*FRAC_PI_4, 5.0*FRAC_PI_6),  true);
        assert_eq!(direction(3.0*FRAC_PI_4, 3.0*FRAC_PI_4),  true);
        assert_eq!(direction(3.0*FRAC_PI_4, 4.0*FRAC_PI_6),  true);
        assert_eq!(direction(3.0*FRAC_PI_4, FRAC_PI_2),      true);
        assert_eq!(direction(3.0*FRAC_PI_4, 2.0*FRAC_PI_6),  true);
        assert_eq!(direction(3.0*FRAC_PI_4, FRAC_PI_4),      true);
        assert_eq!(direction(3.0*FRAC_PI_4, FRAC_PI_6),      false);
        assert_eq!(direction(3.0*FRAC_PI_4, 0.0),            false);
        assert_eq!(direction(3.0*FRAC_PI_4, -FRAC_PI_6),     false);
        assert_eq!(direction(3.0*FRAC_PI_4, -FRAC_PI_4),     false);
        assert_eq!(direction(3.0*FRAC_PI_4, -2.0*FRAC_PI_6), false);
        assert_eq!(direction(3.0*FRAC_PI_4, -FRAC_PI_2),     false);
        assert_eq!(direction(3.0*FRAC_PI_4, -4.0*FRAC_PI_6), false);
        assert_eq!(direction(3.0*FRAC_PI_4, -3.0*FRAC_PI_4), true);
        assert_eq!(direction(3.0*FRAC_PI_4, -5.0*FRAC_PI_6), true);
        assert_eq!(direction(3.0*FRAC_PI_4, -PI),            true);

        // Test circle when t_theta = -3.0*FRAC_PI_4
        assert_eq!(direction(-3.0*FRAC_PI_4, PI),             true);
        assert_eq!(direction(-3.0*FRAC_PI_4, 5.0*FRAC_PI_6),  true);
        assert_eq!(direction(-3.0*FRAC_PI_4, 3.0*FRAC_PI_4),  true);
        assert_eq!(direction(-3.0*FRAC_PI_4, 4.0*FRAC_PI_6),  false);
        assert_eq!(direction(-3.0*FRAC_PI_4, FRAC_PI_2),      false);
        assert_eq!(direction(-3.0*FRAC_PI_4, 2.0*FRAC_PI_6),  false);
        assert_eq!(direction(-3.0*FRAC_PI_4, FRAC_PI_4),      false);
        assert_eq!(direction(-3.0*FRAC_PI_4, FRAC_PI_6),      false);
        assert_eq!(direction(-3.0*FRAC_PI_4, 0.0),            false);
        assert_eq!(direction(-3.0*FRAC_PI_4, -FRAC_PI_6),     false);
        assert_eq!(direction(-3.0*FRAC_PI_4, -FRAC_PI_4),     true);
        assert_eq!(direction(-3.0*FRAC_PI_4, -2.0*FRAC_PI_6), true);
        assert_eq!(direction(-3.0*FRAC_PI_4, -FRAC_PI_2),     true);
        assert_eq!(direction(-3.0*FRAC_PI_4, -4.0*FRAC_PI_6), true);
        assert_eq!(direction(-3.0*FRAC_PI_4, -3.0*FRAC_PI_4), true);
        assert_eq!(direction(-3.0*FRAC_PI_4, -5.0*FRAC_PI_6), true);
        assert_eq!(direction(-3.0*FRAC_PI_4, -PI),            true);
    }
}

