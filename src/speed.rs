/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


use pid::{Pid, ControlOutput};

use rosrust_msg::std_msgs::Float64;
use rosrust_msg::geometry_msgs::Twist;
use rosrust_msg::geometry_msgs::Pose2D;
use rosrust_msg::geometry_msgs::PoseStamped;

use rustros_tf::msg::geometry_msgs::TransformStamped;



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
    prev_tf: Option<TransformStamped>,
}

impl SpeedPidController {

    /// Create a new controller
    pub fn new(c: PidConstants) -> Self {
        let pid = Pid::new(c.kp, c.ki, c.kd, c.p_limit, c.i_limit, c.d_limit, 0.0);
        Self { pid, prev_tf: None}
    }

    // Update the setpoint
    pub fn update_setpoint(&mut self, setpoint: f64) {
        self.pid.setpoint = match setpoint {
            x if x > 1.0 => 1.0,
            x if x < -1.0 => -1.0,
            x => x,
        }
    }

    /// Determine the Steering output based on new measurement
    pub fn update_measurement(&mut self, tf: TransformStamped) -> Float64 {
        self.prev_tf = Some(tf);
        unimplemented!();
    }
}


// Determine the linear and augular speed based on two stamped locations
fn calc_twist(prev: PoseStamped, current: PoseStamped) -> Twist {
    unimplemented!();
}


// Determine the magnitude of the speed based on two stamped locations
fn calc_speed(prev: PoseStamped, current: PoseStamped) -> f64 {
    unimplemented!();
}

