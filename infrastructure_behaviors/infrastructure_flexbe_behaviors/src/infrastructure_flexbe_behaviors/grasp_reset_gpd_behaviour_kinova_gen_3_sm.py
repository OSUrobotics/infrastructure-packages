#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_behaviors.pickandplace_sm import PickAndPlaceSM
from infrastructure_flexbe_states.parameter_ac import ParameterActionClient
from infrastructure_flexbe_states.stage_ac import StageActionClient
from infrastructure_flexbe_states.test_control_state import TestControlState
from infrastructure_flexbe_states.trial_control_state import TrialControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 7/2/2023
@author: Keegan, modified by Kyle
'''
class Grasp_Reset_GPD_Behaviour_Kinova_Gen_3SM(Behavior):
	'''
	Behavior for use with the grasp reset and Kinova Gen 3 arm. Full functionallity, including Grasp Pose Detection.
	'''


	def __init__(self):
		super(Grasp_Reset_GPD_Behaviour_Kinova_Gen_3SM, self).__init__()
		self.name = 'Grasp_Reset_GPD_Behaviour_Kinova_Gen_3'

		# parameters of this behavior
		self.add_parameter('start_data_collection_topic', 'start_data_collection')
		self.add_parameter('parameter_topic', 'set_test_parameters')
		self.add_parameter('reset_topic', 'reset_hardware')
		self.add_parameter('stop_data_collection_topic', 'stop_data_collection')
		self.add_parameter('arm_control_topic', 'my_gen3/start_arm_sequence')
		self.add_parameter('session_info', dict())

		# references to used behaviors
		self.add_behavior(PickAndPlaceSM, 'PickAndPlace')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:217 y:96, x:43 y:266
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:107 y:34
			OperatableStateMachine.add('Test Control',
										TestControlState(session_info=self.session_info),
										transitions={'continue': 'Trial Control', 'failed': 'failed', 'completed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'completed': Autonomy.Off},
										remapping={'trial_info': 'trial_info'})

			# x:575 y:132
			OperatableStateMachine.add('Reset',
										StageActionClient(topic=self.reset_topic),
										transitions={'completed': 'Start Data Collection', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:574 y:35
			OperatableStateMachine.add('Set Test Parameters',
										ParameterActionClient(topic=self.parameter_topic),
										transitions={'completed': 'Reset', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'trial_params': 'trial_params'})

			# x:576 y:225
			OperatableStateMachine.add('Start Data Collection',
										StageActionClient(topic=self.start_data_collection_topic),
										transitions={'completed': 'PickAndPlace', 'failed': 'PickAndPlace'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:338 y:130
			OperatableStateMachine.add('Stop Data Collection',
										StageActionClient(topic=self.stop_data_collection_topic),
										transitions={'completed': 'Trial Control', 'failed': 'Trial Control'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:339 y:34
			OperatableStateMachine.add('Trial Control',
										TrialControlState(),
										transitions={'continue': 'Set Test Parameters', 'failed': 'failed', 'completed': 'Test Control'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'completed': Autonomy.Off},
										remapping={'trial_info': 'trial_info', 'trial_params': 'trial_params'})

			# x:330 y:294
			OperatableStateMachine.add('PickAndPlace',
										self.use_behavior(PickAndPlaceSM, 'PickAndPlace'),
										transitions={'finished': 'Stop Data Collection', 'failed': 'Stop Data Collection'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
