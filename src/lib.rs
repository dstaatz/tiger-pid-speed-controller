/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;

mod errors;
mod speed;


////////////////////////////////////////////////////////////////////////////////


use std::sync::{Arc, Mutex};

use rosrust::Time;
use rosrust_msg::std_msgs::Float64;
use rustros_tf::TfListener;

use errors::*;


pub fn run() -> Result<()> {

    // Get parameters
    // let pid_constants = speed::PidConstants {
    //     kp: rosrust::param("~kp").unwrap().get()?,
    //     ki: rosrust::param("~ki").unwrap().get()?,
    //     kd: rosrust::param("~kd").unwrap().get()?,
    //     p_limit: rosrust::param("~p_limit").unwrap().get()?,
    //     i_limit: rosrust::param("~i_limit").unwrap().get()?,
    //     d_limit: rosrust::param("~d_limit").unwrap().get()?,
    // };

    let pid_constants = speed::PidConstants {
        kp: 1.0,
        ki: 0.0,
        kd: 0.0,
        p_limit: 100.0,
        i_limit: 100.0,
        d_limit: 100.0,
    };

    // Setup controller
    let controller = speed::SpeedPidController::new(pid_constants);
    let setpoint_controller = Arc::new(Mutex::new(controller));
    let update_controller = setpoint_controller.clone();

    // Create publishers
    let drivetrain_pub = rosrust::publish("/tiger_car/speed", 100)?;

    // Register Subscriber to get setpoint
    rosrust::subscribe(
        "/tiger_controller/speed",
        100,
        move |speed: Float64| {
            // Update the new setpoint
            let mut controller = setpoint_controller.lock().unwrap();
            controller.update_setpoint(speed.data);
        }
    )?;

    // Listen for transforms
    let listener = TfListener::new();
    let rate = rosrust::rate(1.0);

    while rosrust::is_ok() {
        // Get updated odom transform
        let tf = listener.lookup_transform("odom", "base", Time::new()).unwrap();
        
        // Determine new output from controller
        let mut controller = update_controller.lock().unwrap();
        let output = controller.update_measurement(tf);
        
        // Publish new control output
        drivetrain_pub.send(output)?;
        
        // Sleep to maintain rate
        rate.sleep();
    }

    Ok(())
}

