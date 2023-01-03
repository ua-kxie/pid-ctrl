use pid_ctrl as pid;

fn main() {
    let mut pid = pid::PidCtrl::new_with_pid(0.5, 0.1, 0.1);
    let mut measurement = 0.0;
    let mut width:usize;
    pid.init(7.5, measurement);

    for _i in 1..21 {
        measurement += pid.step(pid::PidIn::new(measurement, 1.0)).out;
        width = (measurement * 10.0) as usize;

        println!("{measurement:>0$.2}", width);
    }
}