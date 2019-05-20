use crate::static_messages::MUTEX_LOCK_FAIL;
use crate::{Action, ActionServer, ActionServerOnRequest, ServerGoalHandle};
use rosrust::error::Result;
use std::sync::{Arc, Mutex, Weak};
use std::thread;

pub struct SimpleActionServer<T: Action> {
    action_server: ActionServer<T>,
    state: Arc<Mutex<ServerState<T>>>,
}

impl<T: Action> SimpleActionServer<T> {
    pub fn new(
        name: &str,
        on_goal: ActionServerOnRequest<T>,
        on_preempt: ActionServerOnRequest<T>,
    ) -> Result<Arc<Self>> {
        let state = Arc::new(Mutex::new(ServerState::new(on_goal, on_preempt)));
        let on_goal = {
            let state = Arc::downgrade(&state);
            move |handle| {
                if let Some(ref state) = state.upgrade() {
                    state.lock().expect(MUTEX_LOCK_FAIL).on_goal(handle)
                } else {
                    Ok(())
                }
            }
        };
        let on_preempt = {
            let state = Arc::downgrade(&state);
            move |handle| {
                if let Some(ref state) = state.upgrade() {
                    state.lock().expect(MUTEX_LOCK_FAIL).on_preempt(handle)
                } else {
                    Ok(())
                }
            }
        };
        let action_server = ActionServer::new(name, Box::new(on_goal), Box::new(on_preempt))?;
        let server = Arc::new(Self {
            action_server,
            state,
        });

        {
            let server = Arc::downgrade(&server);
            thread::spawn(move || {
                Self::execute_loop(server);
            });
        }

        Ok(server)
    }

    fn register_on_goal_callback(&self, callback: ActionServerOnRequest<T>) {
        self.state.lock().expect(MUTEX_LOCK_FAIL).on_goal_cb = callback;
    }

    fn register_on_preempt_callback(&self, callback: ActionServerOnRequest<T>) {
        self.state.lock().expect(MUTEX_LOCK_FAIL).on_preempt_cb = callback;
    }

    fn on_preempt(&self, preempt: ServerGoalHandle<T>) -> Result<()> {
        self.state
            .lock()
            .expect(MUTEX_LOCK_FAIL)
            .on_preempt(preempt)
    }

    fn on_goal(&self, goal: ServerGoalHandle<T>) -> Result<()> {
        self.state.lock().expect(MUTEX_LOCK_FAIL).on_goal(goal)
    }

    fn execute_loop(server: Weak<Self>) {
        while rosrust::is_ok() {
            let server = match server.upgrade() {
                None => break,
                Some(server) => server,
            };
        }
    }
}

struct ServerState<T: Action> {
    current_goal: Option<ServerGoalHandle<T>>,
    next_goal: Option<ServerGoalHandle<T>>,
    preempt_request: bool,
    new_goal_preempt_request: bool,
    on_goal_cb: ActionServerOnRequest<T>,
    on_preempt_cb: ActionServerOnRequest<T>,
}

impl<T: Action> ServerState<T> {
    fn new(on_goal_cb: ActionServerOnRequest<T>, on_preempt_cb: ActionServerOnRequest<T>) -> Self {
        Self {
            current_goal: None,
            next_goal: None,
            preempt_request: false,
            new_goal_preempt_request: false,
            on_goal_cb,
            on_preempt_cb,
        }
    }

    fn on_goal(&mut self, goal: ServerGoalHandle<T>) -> Result<()> {
        // TODO: execute_condition.acquire

        rosrust::ros_debug!(
            "A new goal {} has been received by the single goal action server",
            goal.goal_id().unwrap_or_default().id,
        );

        if self.is_outdated(&goal) {
            rosrust::ros_err!("This goal was canceled because another goal was received by the simple action server");
            // TODO: execute_condition.release
            return Ok(());
        }

        // TODO: execute_condition.notify
        // TODO: execute_condition.release
        Ok(())
    }

    fn is_outdated(&self, goal: &ServerGoalHandle<T>) -> bool {
        let new_stamp = goal.goal_id().stamp;
        let next_is_newer = self
            .next_goal
            .goal_id()
            .map(|id| id.stamp > new_stamp)
            .unwrap_or(false);
        let current_is_newer = self
            .current_goal
            .goal_id()
            .map(|id| id.stamp > new_stamp)
            .unwrap_or(false);
        next_is_newer || current_is_newer
    }

    fn on_preempt(&mut self, preempt: ServerGoalHandle<T>) -> Result<()> {
        rosrust::ros_debug!("A preempt has been received by the SimpleActionServer");

        if preempt == self.current_goal {
            rosrust::ros_debug!(
                "Setting preempt_request bit for the current goal to TRUE and invoking callback"
            );
            self.preempt_request = true;
            (*self.on_preempt_cb)()
        } else if preempt == self.next_goal {
            rosrust::ros_debug!("Setting preempt request bit for the next goal to TRUE");
            self.new_goal_preempt_request = true;
            Ok(())
        }
    }
}
