/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;

mod errors;
mod speed;


////////////////////////////////////////////////////////////////////////////////


use std::sync::{Arc, Mutex};

// use rosrust::Time;
use rosrust_msg::std_msgs::Float64;
use rosrust_msg::geometry_msgs::Pose2D;
// use rustros_tf::TfListener;

use errors::*;
use speed::{PidConstants, SpeedPidController};


pub fn run() -> Result<()> {

    // Get parameters
    // let pid_constants = PidConstants {
    //     kp: rosrust::param("~kp").unwrap().get()?,
    //     ki: rosrust::param("~ki").unwrap().get()?,
    //     kd: rosrust::param("~kd").unwrap().get()?,
    //     p_limit: rosrust::param("~p_limit").unwrap().get()?,
    //     i_limit: rosrust::param("~i_limit").unwrap().get()?,
    //     d_limit: rosrust::param("~d_limit").unwrap().get()?,
    // };

    let pid_constants = PidConstants {
        kp: 1.0,
        ki: 0.0,
        kd: 0.0,
        p_limit: 100.0,
        i_limit: 100.0,
        d_limit: 100.0,
    };

    let constant_speed: f64 = rosrust::param("~constant_speed").unwrap().get().unwrap_or(0.25);

    // Setup controller
    let controller = SpeedPidController::new(pid_constants, constant_speed);
    let setpoint_controller = Arc::new(Mutex::new(controller));
    let update_controller = setpoint_controller.clone();

    // Create publishers
    let drivetrain_pub = rosrust::publish("/tiger_car/speed", 100)?;

    // Register subscriber to get setpoint
    rosrust::subscribe(
        "/tiger_controller/speed",
        100,
        move |speed: Float64| {
            
            rosrust::ros_info!("Received speed");

            // Update the new setpoint
            let mut controller = setpoint_controller.lock().unwrap();
            controller.update_setpoint(speed);
        }
    )?;

    rosrust::ros_info!("Subscribed to /tiger_controller/speed");

    // Register subscriber to get updated pose from odometry
    rosrust::subscribe(
        "/pose2D",
        100,
        move |pose: Pose2D| {

            println!("RUST: Received pose: {:?}", pose);
            rosrust::ros_info!("ROS: Received pose: {:?}", pose);

            // Update the new pose
            let mut controller = update_controller.lock().unwrap();
            let output = controller.update_pose(pose);

            // Publish new control output
            drivetrain_pub.send(output).unwrap();
        }
    )?;

    rosrust::ros_info!("Subscribed to /pose2D");

    let rate = rosrust::rate(10.0);
    while rosrust::is_ok() {
        rate.sleep();
    }

    // // Listen for transforms
    // let listener = TfListener::new();
    // let rate = rosrust::rate(10.0);

    // std::thread::sleep(std::time::Duration::new(1, 0));

    // while rosrust::is_ok() {
    //     // Get updated odom transform
    //     let tf = listener.lookup_transform("odom", "base", Time::new()).unwrap(); // Probably shouldn't unwrap this
        
    //     // Determine new output from controller
    //     let mut controller = update_controller.lock().unwrap();
    //     let output = controller.update_transform(tf);
        
    //     // Publish new control output
    //     drivetrain_pub.send(output)?;
        
    //     // Sleep to maintain rate
    //     rate.sleep();
    // }

    rosrust::ros_info!("After spin");

    Ok(())
}

