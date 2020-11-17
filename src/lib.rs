/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;

mod errors;
mod speed;


////////////////////////////////////////////////////////////////////////////////


use rosrust_msg::std_msgs::Float64;
use rosrust_msg::geometry_msgs::PoseWithCovarianceStamped;

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

    let speed: f64 = rosrust::param("~speed").unwrap().get()?;
    let speed = Float64 { data: speed };

    // Setup controller
    let controller = speed::SpeedPidController::new(pid_constants);

    // Create publishers
    let drivetrain_pub = rosrust::publish("/tiger_car/speed", 100)?;

    // Register Subscriber
    rosrust::subscribe(
        "amcl_pose",
        100,
        move |pose: PoseWithCovarianceStamped| {

            // Calculate steering output
            let speed = controller.update(pose);

            // Send control outputs
            drivetrain_pub.send(speed.clone()).unwrap();
        }
    )?;

    // Breaks when a shutdown signal is sent
    rosrust::spin();

    Ok(())
}

