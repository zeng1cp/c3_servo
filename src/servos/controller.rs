//! 控制器层：负责任务事务化启动、状态追踪和完成回调。 

use core::ffi::c_void;

use super::config::{ConfigManager, Storage};
use super::engine::{MotionEngine, MotionPlan};
use super::profile::{AngleDeg, CorrectionParams, PhysicalParams, PulseWidthUs, ServoId, ALL_SERVOS, SERVO_COUNT};

#[derive(Clone, Copy)]
pub struct Callback {
    pub func: fn(*mut c_void),
    pub user_data: *mut c_void,
}

#[derive(Clone, Copy)]
pub enum CompletionHook {
    None,
    Ffi(Callback),
}

impl CompletionHook {
    fn call(self) {
        match self {
            CompletionHook::None => {}
            CompletionHook::Ffi(cb) => (cb.func)(cb.user_data),
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum ControlError {
    EmptyMask,
    TargetCountMismatch,
    InvalidState,
    InvalidTarget,
    Engine(super::engine::EngineError),
}

impl From<super::engine::EngineError> for ControlError {
    fn from(e: super::engine::EngineError) -> Self {
        Self::Engine(e)
    }
}

#[derive(Clone, Copy)]
pub enum Target {
    Pwm(PulseWidthUs),
    Angle(AngleDeg),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UnitState {
    Idle,
    Moving { pending_mask: u32 },
    Paused { pending_mask: u32 },
}

#[derive(Clone, Copy)]
struct ServoCommand {
    id: ServoId,
    target_pwm: PulseWidthUs,
}

pub struct ControlUnit {
    mask: u32,
    state: UnitState,
    callback: CompletionHook,
}

impl ControlUnit {
    pub fn new(ids: &[ServoId]) -> Result<Self, ControlError> {
        if ids.is_empty() {
            return Err(ControlError::EmptyMask);
        }
        let mut mask = 0u32;
        for &id in ids {
            mask |= 1 << id.index();
        }
        Ok(Self {
            mask,
            state: UnitState::Idle,
            callback: CompletionHook::None,
        })
    }

    fn contains(&self, id: ServoId) -> bool {
        ((self.mask >> id.index()) & 1) != 0
    }

    fn ids(&self) -> impl Iterator<Item = ServoId> + '_ {
        ALL_SERVOS.iter().copied().filter(|&id| self.contains(id))
    }

    pub fn status(&self) -> UnitState {
        self.state
    }

    pub fn start<S: Storage>(
        &mut self,
        engine: &mut MotionEngine,
        config: &ConfigManager<S>,
        targets: &[Target],
        duration_ms: u32,
        callback: Option<Callback>,
    ) -> Result<(), ControlError> {
        let ids: [ServoId; SERVO_COUNT] = {
            let mut arr = [ServoId::S0; SERVO_COUNT];
            for (i, id) in self.ids().enumerate() {
                arr[i] = id;
            }
            arr
        };
        let count = self.mask.count_ones() as usize;

        if count != targets.len() {
            return Err(ControlError::TargetCountMismatch);
        }

        self.stop(engine);

        let mut commands = [ServoCommand { id: ServoId::S0, target_pwm: PulseWidthUs(0) }; SERVO_COUNT];
        for i in 0..count {
            let id = ids[i];
            let corr = config.get_correction(id);
            let phys = engine.profile(id);
            let target_pwm = Self::convert_target(&targets[i], phys, corr)?;
            commands[i] = ServoCommand { id, target_pwm };
        }

        let mut started_mask = 0u32;
        for cmd in commands.iter().take(count).copied() {
            if let Err(e) = engine.start_motion(
                cmd.id,
                MotionPlan {
                    target_pwm: cmd.target_pwm,
                    duration_ms,
                    easing: None,
                },
            ) {
                for id in ALL_SERVOS {
                    if ((started_mask >> id.index()) & 1) != 0 {
                        engine.stop(id);
                    }
                }
                self.state = UnitState::Idle;
                self.callback = CompletionHook::None;
                return Err(e.into());
            }
            started_mask |= 1 << cmd.id.index();
        }

        self.state = UnitState::Moving {
            pending_mask: self.mask,
        };
        self.callback = callback.map(CompletionHook::Ffi).unwrap_or(CompletionHook::None);
        self.check_immediate_completion(engine);
        Ok(())
    }

    pub fn pause(&mut self, engine: &mut MotionEngine) -> Result<(), ControlError> {
        match self.state {
            UnitState::Moving { pending_mask } => {
                for id in self.ids() {
                    engine.pause(id);
                }
                self.state = UnitState::Paused { pending_mask };
                Ok(())
            }
            _ => Err(ControlError::InvalidState),
        }
    }

    pub fn resume(&mut self, engine: &mut MotionEngine) -> Result<(), ControlError> {
        match self.state {
            UnitState::Paused { pending_mask } => {
                for id in self.ids() {
                    engine.resume(id);
                }
                self.state = UnitState::Moving { pending_mask };
                Ok(())
            }
            _ => Err(ControlError::InvalidState),
        }
    }

    pub fn stop(&mut self, engine: &mut MotionEngine) {
        for id in self.ids() {
            engine.stop(id);
        }
        self.state = UnitState::Idle;
        self.callback = CompletionHook::None;
    }

    pub fn on_completed_mask(&mut self, completed_mask: u32) {
        let pending = match self.state {
            UnitState::Moving { pending_mask } => pending_mask,
            _ => return,
        };

        let remaining = pending & !completed_mask;
        if remaining == 0 {
            self.state = UnitState::Idle;
            let cb = self.callback;
            self.callback = CompletionHook::None;
            cb.call();
        } else {
            self.state = UnitState::Moving {
                pending_mask: remaining,
            };
        }
    }

    pub fn is_active(&self) -> bool {
        !matches!(self.state, UnitState::Idle)
    }

    pub fn is_moving(&self) -> bool {
        matches!(self.state, UnitState::Moving { .. })
    }

    pub fn pending_mask(&self) -> Option<u32> {
        match self.state {
            UnitState::Moving { pending_mask } | UnitState::Paused { pending_mask } => {
                Some(pending_mask)
            }
            UnitState::Idle => None,
        }
    }

    fn convert_target(
        target: &Target,
        phys: PhysicalParams,
        corr: CorrectionParams,
    ) -> Result<PulseWidthUs, ControlError> {
        match target {
            Target::Pwm(pwm) => Ok(PulseWidthUs(corr.apply(pwm.0 as i32, phys.mid_pwm_us))),
            Target::Angle(angle) => {
                if !angle.as_f32().is_finite() {
                    return Err(ControlError::InvalidTarget);
                }
                let raw = phys.angle_to_pwm(*angle);
                Ok(PulseWidthUs(corr.apply(raw.0 as i32, phys.mid_pwm_us)))
            }
        }
    }

    fn check_immediate_completion(&mut self, engine: &MotionEngine) {
        if let UnitState::Moving { pending_mask } = self.state {
            let mut still_pending = pending_mask;
            for id in self.ids() {
                if !engine.is_moving(id) {
                    still_pending &= !(1 << id.index());
                }
            }
            if still_pending == 0 {
                self.state = UnitState::Idle;
                let cb = self.callback;
                self.callback = CompletionHook::None;
                cb.call();
            } else {
                self.state = UnitState::Moving {
                    pending_mask: still_pending,
                };
            }
        }
    }
}
