#!/usr/bin/env python
import rospy
import copy
import std_msgs.msg

from flexbe_core import EventState, Logger

class TrialControlState(EventState):
        '''
        Trial control takes in the trial information from Test control and on a succesful completion starts
        the Data Control. If all trials are completed then it will loop back to Test control. Direction is
        used for determining a successful and unsuccseful outcome for testing purposed but will need to be
        replaced with a different measure of success once it becomes more fleshed out. 
        TODO: More complex information for trials 

        -- rotation  int       TEMPORARY: gives rotation

        ># trial_info          Trial information

        <= continue             All actions completed
        <= failed               Trial control failed to initialize or call something TODO: Proper error checking
        <= completed            All Trials have been succesfully completed, go back to Test control           

        '''

        def __init__(self):
            # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
            super(TrialControlState, self).__init__(outcomes = ["continue", "failed", "completed"], input_keys=["trial_info"], output_keys=["trial_params"])

            # Store state parameters for later use.
            self.num_trials = None
            self._trial_params = []

            self.pub = rospy.Publisher('reset_metadata', std_msgs.msg.Float32, queue_size=10)




        def execute(self, userdata):
            #if trials remain return continue, if not return complete, if direction is 0 return failed
            if(self.num_trials > 0):
                #print(self._number_of_trials)
                print("PARAMTERS IN TRIAL CONTROL STATE")
                print(self._trial_params)
                self.pub.publish(std_msgs.msg.Float32(float(self._trial_params[1])))
                userdata.trial_params = self._trial_params
                self.num_trials -= 1
                return "continue"

            elif(self.num_trials <= 0):
                self.num_trials = None
                return "completed"

            else:
                return "failed"


        def on_enter(self, userdata):
            #Initializes class variable from userdata, has to be done outside of constructor 
            if(self.num_trials is None and userdata.trial_info is not None):
                self._trial_params = []
                self.num_trials = copy.deepcopy(userdata.trial_info["trials"])
                for elem in userdata.trial_info:
                    if elem is not "trials":
                        self._trial_params.append(userdata.trial_info[elem])
                # self._number_of_trials = userdata.number_of_trials
