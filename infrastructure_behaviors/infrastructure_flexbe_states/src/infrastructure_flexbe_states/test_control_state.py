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
        -- session_info file    CSV containing test and trial info as well as which apparatus is being used
                                Format: 
                        
                                        "testbed"
                                        object_index, angle, num_trials
                                        ... (repeat for each test)

                                        OR

                                        "drawer/door"
                                        resistance_value, num_trials
                                        ... (repeat for each test)

        #> trial_info     Information pertaining a test

        <= continue             All actions completed
        <= failed               Test control failed to initialize or call something TODO: Proper error checking
        <= completed            All tests in queue have been succesfully completed, system wide exit            

        '''

        def __init__(self, session_info):
            # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
            super(TestControlState, self).__init__(outcomes = ["continue", "failed", "completed"], output_keys=["trial_info"])

            # # Store state parameters for later use.
            # self._num_trials = num_trials
            # self._num_tests = num_tests

            #added
            self._tests = []
            self._testbed_params = {
                "object" : 0,
                "angle" : 0,
                "trials" : 0,
                "index" : 0
                }
            self._other_params = {
                "resistance" : 0,
                "trials" : 0
            }
            # flexbe already parses the csv file
            raw_info = session_info.split(" ")
            self._apparatus = raw_info[0]
            raw_info = raw_info[1:]

            if(self._apparatus == "testbed"):
                if len(raw_info) == 0:
                    raise IOError("Empty CSV. Be sure to include header line and test information")
                for test in raw_info:
                    parsed_row = test.split(",")
                    self._testbed_params["object"] = float(parsed_row[0])
                    self._testbed_params["angle"] = float(parsed_row[1])
                    self._testbed_params["trials"] = float(parsed_row[2])
                    self._testbed_params["index"] = float(parsed_row[3])
                    self._tests.append(copy.deepcopy(self._testbed_params))

            elif(self._apparatus == "drawer/door"):
                if len(raw_info) == 0:
                    raise IOError("Empty CSV. Be sure to include header line and test information")
                for test in raw_info:
                    parsed_row = test.split(",")
                    self._other_params["resistance"] = float(parsed_row[0])
                    self._other_params["trials"] = float(parsed_row[1])
                    self._tests.append(copy.deepcopy(self._other_params))

            else:
                rospy.loginfo("Invalid header line. Must be 'testbed' or 'drawer/door'")
                raise IOError("Invalid header line")

            self._num_tests = len(self._tests)


        def execute(self, userdata):
            #returns continue if tests remain, if none remain returns completed, if direction is 0 returns failed
            if(self._num_tests > 0):
                userdata.trial_info = self._tests[len(self._tests) - self._num_tests]
                # current_test = self._tests[len(self._tests) - self._num_tests]
                # if(self._apparatus == "testbed"):
                #     userdata.number_of_trials = current_test["trials"]
                #     userdata.parameter_one = current_test["object"]
                #     userdata.parameter_two = current_test["angle"]
                # else:
                #     userdata.number_of_trials = current_test["trials"]
                #     userdata.parameter_one = current_test["resistance"]
                #     userdata.parameter_two = None
                self._num_tests -= 1
                return "continue"

            elif(self._num_tests <= 0):
                return "completed"

            else:
                return "failed"
