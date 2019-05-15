use crate::action_server::status_tracker::StatusTracker;
use crate::static_messages::MUTEX_LOCK_FAIL;
use crate::{
    Action, ActionServerState, FeedbackBody, GoalBody, GoalID, GoalState, GoalStatus, GoalType,
    ResultBody,
};
use std::sync::{Arc, Mutex};

pub struct ServerGoalHandle<T: Action> {
    goal: Option<Arc<GoalType<T>>>,
    action_server: Arc<Mutex<ActionServerState<T>>>,
    status_tracker: Arc<Mutex<StatusTracker<GoalBody<T>>>>,
}

impl<T: Action> ServerGoalHandle<T> {
    pub(crate) fn new(
        action_server: Arc<Mutex<ActionServerState<T>>>,
        status_tracker: Arc<Mutex<StatusTracker<GoalBody<T>>>>,
    ) -> Self {
        let goal = status_tracker.lock().expect(MUTEX_LOCK_FAIL).goal.clone();
        Self {
            goal,
            action_server,
            status_tracker,
        }
    }

    fn log_action(&self, operation: &str) {
        let goal_id = self.goal_id();
        let id = goal_id.id;
        let stamp = goal_id.stamp.seconds();
        rosrust::ros_debug!("{}, id: {}, stamp: {}", operation, id, stamp);
    }

    fn check_goal_presence(&self, operation: &str) -> Result<(), Error> {
        match self.goal {
            Some(_) => Ok(()),
            None => Err(Error::new(format!(
                "Attempt to {} on an uninitialized ServerGoalHandle",
                operation,
            ))),
        }
    }

    #[inline]
    fn set_general(
        &self,
        result: Option<ResultBody<T>>,
        text: Option<&str>,
        action: ActionCommand,
    ) -> Result<(), Error> {
        self.log_action(action.precise_description());

        self.check_goal_presence("set_status")?;

        let mut status_tracker = self.status_tracker.lock().expect(MUTEX_LOCK_FAIL);

        status_tracker.status.state = action
            .do_transition(status_tracker.status.state)
            .map_err(Into::into)?;

        if let Some(text) = text {
            status_tracker.status.text = text.into();
        }

        match action.publish_target() {
            ActionPublishTarget::Status => {
                drop(status_tracker);

                self.action_server
                    .lock()
                    .expect(MUTEX_LOCK_FAIL)
                    .publish_status()
                    .map_err(|err| Error::new(format!("Failed to publish status: {}", err)))
            }
            ActionPublishTarget::Result => {
                status_tracker.handle_destruction_time = rosrust::now();

                let status = status_tracker.status.clone();
                drop(status_tracker);

                self.action_server
                    .lock()
                    .expect(MUTEX_LOCK_FAIL)
                    .publish_result(status, result.unwrap_or_default())
                    .map_err(|err| Error::new(format!("Failed to publish result: {}", err)))
            }
        }
    }

    pub fn set_accepted(&self, text: &str) -> bool {
        self.set_general(None, Some(text), ActionCommand::Accepted)
            .map_err(Error::log)
            .is_ok()
    }

    pub fn set_canceled(&self, result: Option<ResultBody<T>>, text: &str) -> bool {
        self.set_general(result, Some(text), ActionCommand::Canceled)
            .map_err(Error::log)
            .is_ok()
    }

    pub fn set_rejected(&self, result: Option<ResultBody<T>>, text: &str) -> bool {
        self.set_general(result, Some(text), ActionCommand::Rejected)
            .map_err(Error::log)
            .is_ok()
    }

    pub fn set_aborted(&self, result: Option<ResultBody<T>>, text: &str) -> bool {
        self.set_general(result, Some(text), ActionCommand::Aborted)
            .map_err(Error::log)
            .is_ok()
    }

    pub fn set_succeeded(&self, result: Option<ResultBody<T>>, text: &str) -> bool {
        self.set_general(result, Some(text), ActionCommand::Succeeded)
            .map_err(Error::log)
            .is_ok()
    }

    pub fn set_cancel_requested(&self) -> bool {
        // We intentionally do not log the error for this case
        self.set_general(None, None, ActionCommand::CancelRequested)
            .is_ok()
    }

    fn publish_feedback_inner(&self, feedback: FeedbackBody<T>) -> Result<(), Error> {
        self.log_action("Publishing feedback on goal");

        self.check_goal_presence("publish feedback")?;

        let status = self
            .status_tracker
            .lock()
            .expect(MUTEX_LOCK_FAIL)
            .status
            .clone();

        self.action_server
            .lock()
            .expect(MUTEX_LOCK_FAIL)
            .publish_feedback(status, feedback)
            .map_err(|err| Error::new(format!("Failed to publish feedback: {}", err)))
    }

    pub fn publish_feedback(&self, feedback: FeedbackBody<T>) -> bool {
        self.publish_feedback_inner(feedback)
            .map_err(Error::log)
            .is_ok()
    }

    pub fn goal(&self) -> Option<&GoalBody<T>> {
        Some(&self.goal.as_ref()?.body)
    }

    pub fn goal_id(&self) -> GoalID {
        if self
            .check_goal_presence("get a goal id")
            .map_err(Error::log)
            .is_err()
        {
            return GoalID::default();
        }

        self.status_tracker
            .lock()
            .expect(MUTEX_LOCK_FAIL)
            .status
            .goal_id
            .clone()
    }

    pub fn goal_status(&self) -> GoalStatus {
        if self
            .check_goal_presence("get a goal status")
            .map_err(Error::log)
            .is_err()
        {
            return GoalStatus::default();
        }

        self.status_tracker
            .lock()
            .expect(MUTEX_LOCK_FAIL)
            .status
            .clone()
    }
}

struct Error {
    message: String,
}

impl Error {
    fn new(message: String) -> Self {
        Self { message }
    }

    fn log(self) -> Self {
        rosrust::ros_err!("{}", self.message);
        self
    }
}

struct TransitionIssue {
    target: ActionCommand,
    accepted: &'static [GoalState],
    status: GoalState,
}

impl std::fmt::Display for TransitionIssue {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "To transition to {} state, the goal must be in a ",
            match self.target {
                ActionCommand::Accepted => "an active",
                ActionCommand::Canceled => "a canceled",
                ActionCommand::Rejected => "a rejected",
                ActionCommand::Aborted => "an aborted",
                ActionCommand::Succeeded => "a succeeded",
                ActionCommand::CancelRequested => "a requested cancel",
            }
        )?;

        match self.accepted.split_last() {
            None => write!(f, "nonexistent")?,
            Some((item, [])) => write!(f, "{:?}", item)?,
            Some((item2, [item1])) => write!(f, "{:?} or {:?}", item1, item2)?,
            Some((last, most)) => {
                for item in most {
                    write!(f, "{:?}, ", item)?;
                }
                write!(f, "or {:?}", last)?;
            }
        }

        write!(f, " state, it is currently in state: {:?}", self.status)?;
        Ok(())
    }
}

#[derive(Copy, Clone, Debug)]
enum ActionCommand {
    Accepted,
    Canceled,
    Rejected,
    Aborted,
    Succeeded,
    CancelRequested,
}

enum ActionPublishTarget {
    Status,
    Result,
}

impl ActionCommand {
    fn publish_target(self) -> ActionPublishTarget {
        match self {
            ActionCommand::Accepted | ActionCommand::CancelRequested => ActionPublishTarget::Status,
            ActionCommand::Canceled
            | ActionCommand::Rejected
            | ActionCommand::Aborted
            | ActionCommand::Succeeded => ActionPublishTarget::Result,
        }
    }

    fn precise_description(self) -> &'static str {
        match self {
            ActionCommand::Accepted => "Accepting goal",
            ActionCommand::Canceled => "Setting status to canceled on goal",
            ActionCommand::Rejected => "Setting status to rejected on goal",
            ActionCommand::Aborted => "Setting status to aborted on goal",
            ActionCommand::Succeeded => "Setting status to succeeded on goal",
            ActionCommand::CancelRequested => "Transitioning to a cancel requested state on goal",
        }
    }

    fn accepted_states(self) -> &'static [GoalState] {
        match self {
            ActionCommand::Accepted => &[GoalState::Pending, GoalState::Recalling],
            ActionCommand::Canceled => &[
                GoalState::Pending,
                GoalState::Recalling,
                GoalState::Active,
                GoalState::Preempting,
            ],
            ActionCommand::Rejected => &[GoalState::Pending, GoalState::Recalling],
            ActionCommand::Aborted => &[GoalState::Preempting, GoalState::Active],
            ActionCommand::Succeeded => &[GoalState::Preempting, GoalState::Active],
            ActionCommand::CancelRequested => &[GoalState::Pending, GoalState::Active],
        }
    }

    fn do_transition(self, state: GoalState) -> Result<GoalState, Error> {
        match (self, state) {
            (ActionCommand::Accepted, GoalState::Pending) => Ok(GoalState::Active),
            (ActionCommand::Accepted, GoalState::Recalling) => Ok(GoalState::Preempting),

            (ActionCommand::Canceled, GoalState::Pending)
            | (ActionCommand::Canceled, GoalState::Recalling) => Ok(GoalState::Recalled),
            (ActionCommand::Canceled, GoalState::Active)
            | (ActionCommand::Canceled, GoalState::Preempting) => Ok(GoalState::Preempted),

            (ActionCommand::Rejected, GoalState::Pending)
            | (ActionCommand::Rejected, GoalState::Recalling) => Ok(GoalState::Rejected),

            (ActionCommand::Aborted, GoalState::Preempting)
            | (ActionCommand::Aborted, GoalState::Active) => Ok(GoalState::Aborted),

            (ActionCommand::Succeeded, GoalState::Preempting)
            | (ActionCommand::Succeeded, GoalState::Active) => Ok(GoalState::Succeeded),

            (ActionCommand::CancelRequested, GoalState::Pending) => Ok(GoalState::Recalling),
            (ActionCommand::CancelRequested, GoalState::Active) => Ok(GoalState::Preempting),

            (target, status) => {
                let issue = TransitionIssue {
                    accepted: target.accepted_states(),
                    target,
                    status,
                };
                Err(Error::new(format!("{}", issue)))
            }
        }
    }
}