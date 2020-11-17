/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


use std::time::Duration;
use std::cmp::Ordering;

use pid::Pid;
use rosrust_msg::std_msgs::Float64;
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
    constant_speed: f64,
}

impl SpeedPidController {

    /// Create a new controller
    pub fn new(c: PidConstants, constant_speed: f64) -> Self {
        let pid = Pid::new(c.kp, c.ki, c.kd, c.p_limit, c.i_limit, c.d_limit, 0.0);
        Self { pid, prev_tf: None, constant_speed }
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
    pub fn update_measurement(&mut self, tf: TransformStamped) -> Float64 {
        
        match &self.prev_tf {
            None => {
                self.prev_tf = Some(tf);
                Float64 { data: 0.0 }
            },
            Some(prev) => {
                let speed = calc_speed(prev, &tf);
                rosrust::ros_info!("Current measured speed: {}", speed);
                let output = self.pid.next_control_output(speed);
                self.prev_tf = Some(tf);
                Float64 { data: output.output }
            },
        }
    }
}


// Determine the magnitude of the speed in 2d space based on two stamped locations
fn calc_speed(prev: &TransformStamped, cur: &TransformStamped) -> f64 {
    
    let cur_time = Duration::new(cur.header.stamp.sec.into(), cur.header.stamp.nsec.into());
    let prev_time = Duration::new(prev.header.stamp.sec.into(), prev.header.stamp.nsec.into());
    let delta_time = cur_time - prev_time;
    let delta_time = delta_time.as_secs_f64();

    let delta_x = cur.transform.translation.x - prev.transform.translation.x;
    let delta_y = cur.transform.translation.y - prev.transform.translation.y;

    (delta_x*delta_x + delta_y*delta_y).sqrt() / delta_time
}

