#!/usr/bin/env python
import rospy
from armada_flexbe_utilities.msg import GraspPoses, GraspPosesList
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from geometry_msgs.msg import Pose
from dataset_test.srv import XYZ, XYZResponse, XYZRequest
from copy import deepcopy
class XYZState(EventState):
        '''
        TODO

        ># grasp_candidates                             List of grasp candidates message
        #> grasp_waypoints_list                         List of sets of grasp waypoints

        <= continue                                     Calculated list of sets of grasp waypoints
        <= failed                                       Something went wrong

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(XYZState, self).__init__(outcomes = ['continue', 'failed'],
                                                        input_keys = ['grasp_candidates'],
                                                        output_keys = ['grasp_waypoints_list'])

                self._service_topic = '/calc_arm_pose_xyz'
                self._service = ProxyServiceCaller({self._service_topic: XYZ})

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                request = XYZRequest()
                request.request = "NA"
                # request.grasp_msg_list = userdata.grasp_candidates
                # pose1 = Pose()
                # pose1.position.x = 0.44
                # pose1.position.y = 0.00
                # pose1.position.z = 0.23
                # pose1.orientation.x = -.995
                # pose1.orientation.y = -.002
                # pose1.orientation.z = -.0969
                # pose1.orientation.w = 0.00
                # pose2 = deepcopy(pose1)
                # pose2.position.z -= .1
                # pose_msg = GraspPoses()
                # pose_msg.pre = pose1
                # pose_msg.target = pose2
                # pose_msg.post = pose1
                
                # post = GraspPosesList()
                # post.poses = [pose_msg]
                try:
                  service_response = self._service.call(self._service_topic, request)
                #   print(service_response)
                  userdata.grasp_waypoints_list = [service_response.poses] #[pose_msg]#
                  return 'continue'
                except Exception as e:
                  Logger.logwarn(str(e))
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to generate a list of grasp waypoint sets...' )

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Nothing to do in this state.

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                # rospy.wait_for_service(self._service_topic)
                pass

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Nothing to do in this state.