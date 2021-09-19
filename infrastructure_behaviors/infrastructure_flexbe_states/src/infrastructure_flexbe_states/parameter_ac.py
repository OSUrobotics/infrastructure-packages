#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import rosnode
import csv
import copy

# example import of required action
from infrastructure_msgs.msg import TestParametersAction, TestParametersGoal


class ParameterActionClient(EventState):
    '''
    parameter_ac is an action client state that will make a call to an action server.
    This is intended to serve as a template for future states that need to communicate with 
    action server. The topic given should correspond to an action server and it right now only 
    works for StageAction action messages but in the future it will have any custom messages needed
    once they are more finalized. Currently just sends true or false to an action server and
    either returns complete or failed based on the response.
    TODO: Proper error checking other custom messages
     
    <= continue             All actions completed, data collection started
    <= failed               Data control failed to initialize or call something TODO: Proper error checking          

    '''


    def __init__(self, topic):
        # See example_state.py for basic explanations.
        super(ParameterActionClient, self).__init__(outcomes = ['completed', 'failed'])

        self._topic = topic
        self._client = ProxyActionClient({self._topic: TestParametersAction})

	# It may happen that the action client fails to send the action goal.
        self._error = False

        #temporary user interface for inputting test parameters
        self.trial_count = 0
        self.test_count = 0
        self.read_csv = False
        self.tests = []
        self.params = {
                "object" : 0,
                "angle" : 0,
                "trials" : 0
                }


    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'

        #otherwise get the result and perform next steps
        if(self._client.has_result(self._topic)):
            result = self._client.get_result(self._topic)
            status = result.result
            if(status == 0):
                print("Completed Succesfully")
                return 'completed'
            else:
                print("Error Thrown")
                return 'failed'


    def on_enter(self, userdata):
        #Creating the goal to send for testing
        goal = TestParametersGoal()
        
        #for testbed. Name of file to be read: parameters.csv
        self.trial_count = self.trial_count + 1
        if(rosnode.rosnode_ping("testbed_controller", max_count=10)):
            if(not self.read_csv):
                with open('parameters.csv', mode='r') as f:
                    reader = csv.reader(f, delimiter=' ', quotechar='|')
                    for row in reader:
                        parsed_row = row[0].split(",")
                        self.params["object"] = float(parsed_row[0])
                        self.params["angle"] = float(parsed_row[1])
                        self.params["trials"] = float(parsed_row[2])
                        self.tests.append(copy.deepcopy(self.params))
                self.read_csv = True
            current_test = self.tests[self.test_count]
            if(self.trial_count == current_test["trials"]):
                self.test_count = self.test_count + 1
                self.trial_count = 0
            goal.parameters = [current_test["object"], current_test["angle"]]

        #for door/drawer
        else:
            goal.parameters = [1,2,3]

        #error checking in case communication cant be established
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the start data collection command:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
		# Makes sure that the action is not running when leaving this state.
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')


        
