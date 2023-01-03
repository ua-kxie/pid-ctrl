#[test]
fn api() {
    let mut pid = pid_ctrl::PidCtrl::default();
    pid.kp.set_scale(3.0);
    pid.ki.set_scale(2.0);
    pid.kd.set_scale(1.0);
    let mut pid1 = pid_ctrl::PidCtrl::new_with_pid(3.0, 2.0, 1.1);
    pid1.kd.set_scale(1.0);
    assert_eq!(pid, pid1);
}

#[test]
fn example() {
    let mut pid = pid_ctrl::PidCtrl::new_with_pid(3.0, 2.0, 1.0);

    let setpoint = 5.0;
    let prev_measurement = 0.0;
    // calling init optional. Setpoint and prev_measurement set to 0.0 by default.
    // Recommended to avoid derivative kick on startup
    pid.init(setpoint, prev_measurement);

    let measurement = 0.0;
    let time_delta = 1.0;
    assert_eq!(
        pid.step(pid_ctrl::PidIn::new(measurement, time_delta)), 
        pid_ctrl::PidOut::new(15.0, 10.0, 0.0, 25.0)
    );

    // changing pid constants
    pid.kp.set_scale(4.0);
    assert_eq!(
        pid.step(pid_ctrl::PidIn::new(measurement, time_delta)), 
        pid_ctrl::PidOut::new(20.0, 20.0, 0.0, 40.0)
    );

    // setting symmetrical limits around zero
    pid.kp.limits.set_limit(10.0);
    assert_eq!(
        pid.step(pid_ctrl::PidIn::new(measurement, time_delta)), 
        pid_ctrl::PidOut::new(10.0, 30.0, 0.0, 40.0)
    );

    let time_delta = 0.5;
    assert_eq!(
        pid.step(pid_ctrl::PidIn::new(measurement, time_delta)), 
        pid_ctrl::PidOut::new(10.0, 35.0, 0.0, 45.0)
    );

    // setting upper limits returns error if new value conflicts with lower limit
    pid.ki.limits.try_set_upper(28.0).unwrap();  
    assert_eq!(
        pid.step(pid_ctrl::PidIn::new(measurement, time_delta)), 
        pid_ctrl::PidOut::new(10.0, 28.0, 0.0, 38.0)
    );

    // time_delta gets clamped to Float::epsilon() - Float::infinity()
    let measurement = 1.0;
    let time_delta = -7.0;
    pid.kd.set_scale(num_traits::float::FloatCore::epsilon());
    assert_eq!(pid.step(
        pid_ctrl::PidIn::new(measurement, time_delta)), 
        pid_ctrl::PidOut::new(10.0, 28.0, -1.0, 37.0)
    );

    // configure setpoint directly
    pid.setpoint = 1.0;
    assert_eq!(pid.step(
        pid_ctrl::PidIn::new(measurement, time_delta)), 
        pid_ctrl::PidOut::new(0.0, 28.0, 0.0, 28.0)
    );
}