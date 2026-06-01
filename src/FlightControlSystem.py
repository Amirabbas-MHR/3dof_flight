import numpy as np
from abc import ABC, abstractmethod

class FCS:

	def __init__(self, modes):
		"""
		Flight Control System main object.
		computes the controller input vector, based on the current flight mode (current_mode) [set via FCS.set_mode]

		args: dict modes: a dict of mode names as keys and corresponding FlightMode objects as values.
		returns: np.array U: calculated controll input
    	"""
		self.modes = modes
		self.current_mode = None


	def set_mode(self, mode_name):
		"""
    	given a mode_name, sets the current FCS mode to self.modes[mode_name]
		"""
		self.current_mode = self.modes[mode_name]

	def compute(self, t, X, X_ref):
		"""
		computes the controller input vector, based on current state (X), desired state (X_ref) in time t,
		under the rules of the current flight mode (current_mode)
		"""
		U = self.current_mode.compute(t, X, X_ref)
		return U


class FlightMode:

	def __init__(self, controller_modules, ff=0):
		"""
		Flight Mode object. Given a set of tuned controller modules for this flight mode, computes the controller input vector.
		then, returns the computed vector plus a given feed forward (ff) vector [useful for perturbed situations, where U_trim is added to controller results]
		*Note that, the controller_module objects in controller_module list, must be in order.
		The nth controller_module is used to calculate the nth control elemnt U[n]
		Thus, len(FlightMode.controller_modules) = len(U)
		"""
		self.controller_modules = controller_modules
		self.ff = ff


	def compute(self, t, X, X_ref):
		"""
		computes the controller input vector, based on current state (X), desired state (X_ref) in time t, for the given controller sequence.
		"""
		
		return np.array([
		    ctrl.compute(t, X, X_ref)
		    for ctrl in self.controller_modules
		]) + self.ff

class ControllerModule:
    def __init__(self, controller, state_map=None, ref_map=None):
        """
        controller: instance of Controller
        state_map: function X -> x_sub (a sub vector of X, to select which elements go to controller)
        ref_map: function X_ref -> ref_sub (a sub vector of X_ref, to select which elements go to controller)
        """
        self.controller = controller
        self.state_map = state_map if state_map is not None else lambda X: X
        self.ref_map = ref_map if ref_map is not None else lambda X_r: X_r

    def compute(self, t, X, X_ref):
        x_sub = self.state_map(X)
        ref_sub = self.ref_map(X_ref)
        return self.controller.compute(t, x_sub, ref_sub)

    def reset(self):
        self.controller.reset()

class Controller(ABC):

	def __init__(self):
		"""
		Controller abstract class. different controllers (such as PID, LQR, ...) inherit this class and 
		implemnt it's methods for generalization of controller objects (stateless, statefull, ...)

		**BEWARE**: if the controller has t or dt in it's calculations, it should be synced with variant step_size of the ODE solver.
			for example, when estimating the integral part of a PID, dt should not be taken constant,
			but it must be calculated from the difference of current t and the previous t.
			Also, here, frequency of the controller signal update is equal to dynamic calculations, yet in real systems,
			digital controllers work with way higher time-steps than the integrator time-step (e.g. 50 Hz update)"""
		pass

	def reset(self):
		"""Reset internal states (does not have anything to do with anti-windups! Only makes re-using the controller object possible.)"""
		pass

	@abstractmethod
	def compute(self, t, X, X_ref):
		"""Return control input U"""
		pass


class PDController(Controller):

	def __init__(self, Kp, Kd):
		"""
		Proportional-Derivative controller implementation of abstract Controller class.
		"""
		super().__init__()
		self.Kp = Kp
		self.Kd = Kd

	def compute(self, t, x: np.array, x_ref: np.array):
		"""
		**important assumption : x is a vector of the control value and it's derivative, both given to the controller via x vector.
		Meaning that this PD controller doesen't estimate the derivative itself(delta_error / delta_t), but it uses the provided derivative in x vector.
		This, makes it a state feedback PD controller.
		"""

		error = x_ref - x

		# assuming x = [value, value_dot]
		u = self.Kp * error[0] + self.Kd * error[1]

		return u


class PIDController(Controller):

	def __init__(self, Kp, Ki, Kd, u_min=None, u_max=None):
		"""
		Proportional-Integral-Derivative controller implementation of abstract Controller class.
		"""
		super().__init__()

		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd

		self.integral = 0.0
		self.prev_t = None

		self.u_min = u_min
		self.u_max = u_max


	def reset(self):
		"""call in case of re-use."""
		self.integral = 0.0
		self.prev_t = None


	def compute(self, t, x, x_ref):
		"""
		**important assumption : x is a vector of the control value and it's derivative, both given to the controller via x vector.
		Meaning that this PID controller doesen't estimate the derivative itself(delta_error / delta_t), but it uses the provided derivative in x vector.
		But for the integral part, it uses estimation of dt * error.

		Note: Compatible with variant time-step, because it calculates dt each time (difference of the current and previous provided t, stored in dt)
		"""
		# assuming x = [value, value_dot]
		error = x_ref - x

		if self.prev_t is None:
		    dt = 0.0

		else:
		    dt = t - self.prev_t

		# Integral update
		self.integral += error * dt

		# Derivative
		derivative = error[1]

		# Raw control
		u = self.Kp * error[0] + self.Ki * self.integral + self.Kd * derivative

		# Anti-windup via clamping
		if self.u_min is not None and u < self.u_min:
		    u = self.u_min
		    self.integral -= error[0] * dt  # rollback

		elif self.u_max is not None and u > self.u_max:
		    u = self.u_max
		    self.integral -= error[0] * dt

		self.prev_t = t

		return u


class ZeroController(Controller):

	def __init__(self, constant_signal):
		"""
		Dummy controller for 'no controller' tests. requieres full X and X_ref and returns zero as result.
		always returns the given constat signal as U_cmd.
		"""
		super().__init__()
		self.constant_signal = constant_signal
		

	def compute(self, t, x, x_ref):

		return self.constant_signal