/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;

mod errors;
mod speed;


////////////////////////////////////////////////////////////////////////////////


use std::sync::{Arc, Mutex};

use rosrust_msg::std_msgs::Float64;
use rosrust_msg::geometry_msgs::Pose2D;

use errors::*;
use speed::{PidConstants, SpeedPidController};


pub fn run() -> Result<()> {

    let pid_constants = PidConstants {
        kp: 6.0,
        ki: 0.1,
        kd: 0.0,
        p_limit: 100.0,
        i_limit: 100.0,
        d_limit: 100.0,
    };

    let constant_speed: f64 = rosrust::param("~constant_speed").unwrap().get().unwrap_or(0.5);

    // Setup controller
    let controller = SpeedPidController::new(pid_constants, constant_speed);
    let setpoint_controller = Arc::new(Mutex::new(controller));
    let update_controller = setpoint_controller.clone();

    // Create publishers
    let drivetrain_pub = rosrust::publish("/tiger_car/speed", 100)?;

    // Register subscriber to get setpoint
    let _s0 = rosrust::subscribe(
        "/tiger_controller/speed",
        100,
        move |speed: Float64| {

            // Update the new setpoint
            let mut controller = setpoint_controller.lock().unwrap();
            controller.update_setpoint(speed);
        }
    )?;

    rosrust::ros_info!("Subscribed to /tiger_controller/speed");

    // Register subscriber to get updated pose from odometry
    let _s1 = rosrust::subscribe(
        "/laser_scan_matcher/pose2D",
        100,
        move |pose: Pose2D| {

            // Update the new pose
            let mut controller = update_controller.lock().unwrap();
            let output = controller.update_pose(pose);

            // Publish new control output
            drivetrain_pub.send(output).unwrap();
        }
    )?;

    rosrust::ros_info!("Subscribed to /laser_scan_matcher/pose2D");

    // Just chill until program exits
    rosrust::spin();
    Ok(())
}

