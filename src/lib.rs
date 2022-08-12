//! A proportional-integral-derivative (PID) controller.
#![no_std]
use num_traits::{float::FloatCore};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// #[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Display, Default)]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum PidError {
    LimitOutBound,
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Limits<T: FloatCore + core::default::Default> {
    lower: T,
    upper: T,
}

impl<T: FloatCore + core::default::Default> Limits<T> {
    fn new() -> Self {
        Limits{lower: T::neg_infinity(), upper: T::infinity()}
    }

    fn clamp(&self, val: T) -> T {
        val.min(self.upper).max(self.lower)
    }

    pub fn set_limit(&mut self, val: T) -> &mut Self {
        self.lower = -val.abs();
        self.upper = val.abs();
        self
    }

    pub fn try_set_upper(&mut self, val: T) -> Result<&mut Self, PidError> {
        if self.lower <= val {
            self.upper = val;
            Ok(self)
        }
        else {
            Err(PidError::LimitOutBound)
        }
    }

    pub fn try_set_lower(&mut self, val: T) -> Result<&mut Self, PidError> {
        if self.upper >= val {
            self.lower = val;
            Ok(self)
        }
        else {
            Err(PidError::LimitOutBound)
        }
    }
}

impl<T: FloatCore + core::default::Default> Default for Limits<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct KPTerm<T: FloatCore + core::default::Default> {
    pub limits: Limits<T>,
    scale: T,
}

impl<T:FloatCore + core::default::Default> KPTerm<T> {
    pub fn new() -> Self {
        KPTerm::default()
    }
    pub fn set_scale(&mut self, val: T) -> &mut Self {
        self.scale = val;
        self
    }
    pub fn step(&self, offset: T) -> T {
        self.limits.clamp(self.scale * offset)
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct KITerm<T: FloatCore + core::default::Default> {
    pub limits: Limits<T>,
    scale: T,
    pub accumulate: T
}

impl<T:FloatCore + core::default::Default> KITerm<T> {
    pub fn new() -> Self {
        KITerm::default()
    }
    pub fn set_scale(&mut self, val: T) -> &mut Self {
        self.scale = val;
        self
    }
    pub fn step(&mut self, offset: T, tdelta: T) -> T {
        let i = self.limits.clamp(self.scale * offset * tdelta + self.accumulate);
        self.accumulate = i;
        i
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct KDTerm<T: FloatCore + core::default::Default> {
    pub limits: Limits<T>,
    scale: T,
    pub prev_measurement: T
}

impl<T:FloatCore + core::default::Default> KDTerm<T> {
    pub fn new() -> Self {
        KDTerm::default()
    }
    pub fn set_scale(&mut self, val: T) -> &mut Self {
        self.scale = val;
        self
    }
    pub fn step(&mut self, measurement: T, tdelta: T) -> T {
        let d = self.limits.clamp(self.scale * (self.prev_measurement - measurement) / tdelta);
        self.prev_measurement = measurement;
        d
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PidCtrl <T: FloatCore + core::default::Default> {
    pub kp: KPTerm<T>,
    pub ki: KITerm<T>,
    pub kd: KDTerm<T>,
    pub limits: Limits<T>,
    
    pub setpoint: T,
}

impl<T: FloatCore + core::default::Default> PidCtrl<T>
    {
        pub fn new() -> Self {
            PidCtrl::default()
        }

        pub fn new_with_pid(p: T, i: T, d: T) -> Self {
            Self{
                kp: KPTerm{limits:Limits::new(), scale: p}, 
                ki: KITerm{limits:Limits::new(), scale: i, accumulate:T::zero()}, 
                kd: KDTerm{limits:Limits::new(), scale: d, prev_measurement:T::zero()}, 
                limits: Limits::new(), setpoint: T::zero(),
            }
        }

        pub fn init(&mut self, setpoint: T, prev_measurement: T) -> &mut Self {
            self.setpoint = setpoint;
            self.kd.prev_measurement = prev_measurement;
            self
        }

        pub fn step(&mut self, input: PidIn<T>) -> PidOut<T> {
            let offset = self.setpoint - input.measurement;
            let p = self.kp.step(offset);
            let i = self.ki.step(offset, input.tdelta);
            let d = self.kd.step(input.measurement, input.tdelta);
            PidOut::new(p, i, d, self.limits.clamp(p + i + d))
        }
    }

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PidIn <T: FloatCore + core::default::Default> {
    measurement: T,
    tdelta: T,
}

impl<T: FloatCore + core::default::Default> PidIn<T> {
        pub fn new(measurement:T, tdelta:T) -> Self {
            let tdelta_clamped = tdelta.min(T::infinity()).max(T::epsilon());
            PidIn{measurement, tdelta: tdelta_clamped}
        }
    }

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PidOut <T: FloatCore + core::default::Default> {
    p: T,
    i: T,
    d: T,
    out: T,
}

impl<T: FloatCore + core::default::Default> PidOut<T> {
        pub fn new(p:T, i:T, d:T, out:T) -> Self {
            Self{p, i, d, out}
        }
    }

#[cfg(test)]
mod tests {
    #[test]
    fn limits_error() {
        let mut pid = super::PidCtrl::new_with_pid(3.0, 2.0, 1.0);
        pid.kp.limits.try_set_lower(10.0).unwrap();
        assert_eq!(super::PidError::LimitOutBound, pid.kp.limits.try_set_upper(5.0).unwrap_err());
    }

    #[test]
    fn kp() {
        let kp = 0.2;
        let measurement = 0.0;
        let setpoint = 1.0;

        let mut pid = super::PidCtrl::default();
        pid.init(setpoint, 0.0);
        pid.kp.set_scale(kp);

        let kpterm = kp * (setpoint - measurement);

        let inp = super::PidIn::new(measurement, 1.0);
        assert_eq!(pid.step(inp), super::PidOut::new(kpterm, 0.0, 0.0, kpterm));
    }

    #[test]
    fn ki() {
        let ki = 1.0;
        let measurement = 0.0;
        let setpoint = 1.0;
        let td = 1.0;

        let mut pid = super::PidCtrl::default();
        pid.init(setpoint, 0.0);
        pid.ki.set_scale(ki);

        let mut kiterm = 0.0;

        kiterm += ki * (setpoint - measurement) * td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, kiterm, 0.0, kiterm));

        kiterm += ki * (setpoint - measurement) * td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, kiterm, 0.0, kiterm));
    }

    #[test]
    fn kd() {
        let kd = 1.0;
        let measurement = 0.0;
        let setpoint = 1.0;
        let td = 1.0;
        
        let mut prev = 0.0;

        let mut pid = super::PidCtrl::default();
        pid.init(setpoint, prev);
        pid.kd.set_scale(kd);

        let mut kdterm = kd * (measurement - prev) / td;
        prev = measurement;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, 0.0, kdterm, kdterm));

        kdterm = kd * (measurement - prev) / td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, 0.0, kdterm, kdterm));
    }
}
