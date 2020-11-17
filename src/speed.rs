/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


use pid::{Pid, ControlOutput};

use rosrust_msg::std_msgs::Float64;

use rosrust_msg::geometry_msgs::PoseWithCovarianceStamped;


#[derive(Debug, Clone, PartialEq)]
pub struct PidConstants {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub p_limit: f64,
    pub i_limit: f64,
    pub d_limit: f64,
}

pub struct SpeedPidController {
    pid: Pid<f64>,
}

impl SpeedPidController {

    /// Create a new controller
    pub fn new(c: PidConstants) -> Self {
        let pid = Pid::new(c.kp, c.ki, c.kd, c.p_limit, c.i_limit, c.d_limit, 0.0);
        Self { pid }
    }

    /// Determine the Steering output
    pub fn update(&self, pose: PoseWithCovarianceStamped) -> Float64 {
        // TODO
        Float64 { data: 1.0 }
    }
}

