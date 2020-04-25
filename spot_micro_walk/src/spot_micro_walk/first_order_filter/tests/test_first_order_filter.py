'''Tests the spot micro stick figure and spot micro leg classes'''

import unittest
import numpy as np
from ..fof import FirstOrderFilter

class TestFirstOrderFilter(unittest.TestCase):
    '''Tests first order filter class'''

    def test_initial_condition(self):
        '''Test command setting'''
        dt = 0.05
        tau = 1
        x0 = 10

        fof = FirstOrderFilter(dt=dt,tau=tau,x0=x0)

        out = fof.get_output()

        self.assertEqual(out,x0)

    def test_reset(self):
        '''Test resetting state'''
        dt = 0.05
        tau = 1
        x0 = 0

        fof = FirstOrderFilter(dt=dt,tau=tau,x0=x0)
        # Set a command and run for 100 steps
        fof.set_command(10)
        for i in range(100):
            fof.run_timestep()

        # Verify output does not equal beginning state
        self.assertNotEqual(x0,fof.get_output())

        # Reset state
        rst_state = 20
        fof.reset_state(rst_state)

        # Verify state was set
        self.assertEqual(rst_state,fof.get_output())

    def test_command(self):
        '''Test setting a command, verify characteristics of first order response'''
        
        # After 1 time constant, the output should be about 63.2% of the command value
        tol = 0.01
        expected = 0.632

        dt = 0.05
        tau = 1
        x0 = 0
        cmd = 1
        
        fof = FirstOrderFilter(dt=dt,tau=tau,x0=x0)

        # Create time vector and initialize output vector
        time = np.arange(0,tau+dt,dt)
        out = []

        # Set command
        fof.set_command(cmd)

        # Get first value
        out.append(fof.get_output())

        # Run filter
        for t in time[1:]:
            out.append(fof.run_timestep_and_get_output())

        # Get last value
        val = out[-1]

        # Initialize test result to False, failure
        result = False

        # Test if output value is within tolerance of expected value
        if (abs(val-expected) < tol):
            result = True

        self.assertTrue(result)
