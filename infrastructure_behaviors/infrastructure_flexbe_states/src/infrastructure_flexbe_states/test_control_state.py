#!/usr/bin/env python
import rospy
import copy

from flexbe_core import EventState, Logger


class TestControlState(EventState):
        '''
        Test control takes in the test information through parameters and outputs them as userdata.
        In the future a more complex state or function for parsing test info for complicated states
        will be necessary. Direction is used for determining a successful and unsuccseful outcome for
        testing purposed but will need to be replaced with a different measure of success once it becomes 
        more fleshed out. 
        TODO: find a better format for test and trial information
         - session_info works (pass in ~/infrastructure_system/parameters.csv)
         - can get rid of num_trials and num_tests (num_tests = length of self._tests[], num_trials for each test are in params)
         - format of csv file:
            "apparatus name",
            if testbed: "object index", "reset angle", "num trials" (repeat for each test)
            if door/drawer: "resistance", "num trials" (repeat for each test)
         - for each test in array, pass on parameter information to trial_control_state.py
         - can get rid of additional counter and handler in parameter_ac.py

        -- num_trials int       Number of trials for each test TODO: Replace with array with more info 
        -- num_tests  int       Number of tests TODO: Replace with array with more info 

        #> number_of_trials     Number of trials for test

        <= continue             All actions completed
        <= failed               Test control failed to initialize or call something TODO: Proper error checking
        <= completed            All tests in queue have been succesfully completed, system wide exit            

        '''

        def __init__(self, num_trials, num_tests, session_info):
            # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
            super(TestControlState, self).__init__(outcomes = ["continue", "failed", "completed"], output_keys=["number_of_trials"])

            # Store state parameters for later use.
            self._num_trials = num_trials
            self._num_tests = num_tests

            #added
            self._tests = []
            self._params = {
                "object" : 0,
                "angle" : 0,
                "trials" : 0
                }
            for test in session_info.split():
                parsed_row = test.split(",")
                self._params["object"] = float(parsed_row[0])
                self._params["angle"] = float(parsed_row[1])
                self._params["trials"] = float(parsed_row[2])
                self._tests.append(copy.deepcopy(self._params))


        def execute(self, userdata):
            #returns continue if tests remain, if none remain returns completed, if direction is 0 returns failed
            if(self._num_tests > 0):
                self._num_tests -= 1
                userdata.number_of_trials = self._num_trials
                return "continue"

            elif(self._num_tests <= 0):
                return "completed"

            else:
                return "failed"
