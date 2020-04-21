'''Class definition for a first order filter'''

class FirstOrderFilter:
    '''Encapsulates a first order filter object. Assumes to be sampled at a fixed timestep. 
    Can be used to extract a first order response from a command
    
    Attributes:
        _dt: Sample time in seconds first order filter is assumed to run at
        _tau: Time constant of first order filter, in seconds
        _state: Internal state of first order filter object
        _cmd: Command input
        _alpha: Derived constant for implementation of first order filter in discrete time


        This filter assumes the following equations:
        - y[i] is the current output, current state
        - y[i-1] is the previous output, previous state
        - u[i] is the current command

        y[0], u[0] = x0

        y[i] =  (1 - alpha) * y[i-1]  + alpha * u[i]

        alpha = dt / (tau + dt)
    '''

    def __init__(self, dt, tau=1, x0=0):
        '''Constructor, initializes sample time, time constant and initial conditions

        Args:
            dt: Sample time in seconds. Filter is assumed to be continuously called at this interval
            tau: Time constant in seconds
            x0: State and output initial condition
        '''

        self._dt = dt
        self._tau = tau
        self._state = x0
        self._cmd = x0
        self._alpha = self._dt / (self._tau + self._dt)


    def set_command(self,u):
        '''Set command'''
        self._cmd = u
    
    def run_timestep_and_get_output(self):
        ''' Run a time step, calculate and update states, return an a output'''

        self.run_timestep()

        return self._state

    def run_timestep(self):
        '''Run a timestep, return no output'''
        # Convenience variables
        a = self._alpha
        y_prev = self._state
        u = self._cmd
        
        # Update equation
        y = (1-a)*y_prev  + a*u
        
        self._state = y

    def get_output(self):
        '''Return current state'''
        return self._state

    def reset_state(self,x0):
        '''Reset filter state to inputted state'''
        self._state = x0