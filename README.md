# hello_world
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

ROBOT_FRAME = 'base_link'
GLOBAL_FRAME = 'map'
map_folder_name = "solustar"

HOME_STR = os.environ["HOME"]
waypoint_file_uri = HOME_STR + "/SDR/src/sdr_slam/maps/"
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
navigation_launch_node = None

spray_pub = rospy.Publisher('/nodejs/spray', UInt8, queue_size=1)
spray_direction_pub = rospy.Publisher('/nodejs/spray_direction', UInt8, queue_size=1)
move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

init_pose_pub = rospy.Publisher("/sdr/slam/initialpose", PoseWithCovarianceStamped, queue_size=1)

curr_pose_estimate_topic = "/sdr/slam/amcl_pose"
MISSION_PLANNER_STATE_TOPIC = "/sdr/mission_planner/state" # UInt8: 0 is Normal, 1 is Pause, 2 is Cancel and Stop, 3 is Cancel and go to origin
feedback_pub = rospy.Publisher("/sdr/mission_planner/feedback", Int8, queue_size=1) # Int8: -1 is finishing, 0-N is going to waypoint x

def load_rosparams():
    global ROBOT_FRAME
    global GLOBAL_FRAME
    global HOME_STR
    global waypoint_file_uri
    global navigation_launch_node
    global config

    map_folder_name = rospy.get_param("~map_name")
    GLOBAL_FRAME = rospy.get_param("~goal_frame")
    ROBOT_FRAME = rospy.get_param("~robot_frame")
    waypoint_file = rospy.get_param("~waypoint_file")    

    waypoint_file_uri += map_folder_name + "/" + waypoint_file
    with open(waypoint_file_uri, 'r') as stream:
        config = load(stream, Loader=Loader)

        
    navigation_launch_file_uri = HOME_STR + "/SDR/src/sdr_navigation/launch/sdr_navigation.launch"
    navigation_args = ["localize_method:=amcl"]
    map_server_launch_file_uri = HOME_STR + "/SDR/src/sdr_slam/launch/amcl.launch"
    map_server_args = ["map_folder:="+map_folder_name]
    navigation_launch_node = roslaunch.parent.ROSLaunchParent(uuid, [(map_server_launch_file_uri, map_server_args), 
                                                              (navigation_launch_file_uri, navigation_args)])
class LoadNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):

        navigation_launch_node.start()
        
        global move_base_client
        rospy.logwarn("[SDR_MISSION_PLANNER] Waiting for MoveBase Server to be online....")
        move_base_client.wait_for_server()
        rospy.loginfo("[SDR_MISSION_PLANNER] MoveBase Server is running. Proceeding execute mission.")
        return 'success'

class LoadWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        global config
        rospy.loginfo(config)
        if 'waypoints' in config and 'spray_mode' in config:
            global init_pose_pub
            init_pose_msg = PoseWithCovarianceStamped()
            init_pose_msg.header = Header()
            init_pose_msg.header.frame_id = 'map'
            origin = config['waypoints']['origin'][0]

            init_pose_msg.pose.pose.position.x = origin[0]

            init_pose_msg.pose.pose.position.y = origin[1]

            init_pose_msg.pose.pose.position.z = origin[2]
          
            init_pose_msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(*origin[3]))
            init_pose_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0,0, 0, 0, 0, 0, 0.1]
            rospy.loginfo("Initial Pose provided: " + str(origin))
            init_pose_pub.publish(init_pose_msg)
            return 'success'
        else:
            return 'failure'

class GetNextWaypointState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loading_success', 'loading_failure', 'mission_finished'],
                            input_keys=['waypoint_status'],
                            output_keys=['waypoint_status'])

    def execute(self, userdata):
        global config
        global feedback_pub

        curr_idx = 0

        if 'curr_idx' in userdata.waypoint_status:
            curr_idx = userdata.waypoint_status['curr_idx'] + 1

            feedback_msg = Int8()

            if curr_idx >= len(config['waypoints']['spray_locations']):
                # double check spray off at the end
                spray_msg = UInt8()
                spray_msg.data = 0
                spray_pub.publish(spray_msg)

                # check if nozzle is back to center at the end
                spray_direction_msg = UInt8()
                spray_direction_msg.data = 0
                spray_direction_pub.publish(spray_direction_msg)

                # Send end index -1 as feedback
                feedback_msg.data = -1
                feedback_pub.publish(feedback_msg)
                return 'mission_finished'

            feedback_msg.data = curr_idx
            feedback_pub.publish(feedback_msg)
        else:
            curr_idx = 0

        userdata.waypoint_status['curr_idx'] = curr_idx
        userdata.waypoint_status['waypoint'] = config['waypoints']['spray_locations'][curr_idx]

        return 'loading_success'

class GetNextSprayState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loading_success', 'loading_failure'],
                            input_keys=['waypoint_status', 'spray_status'],
                            output_keys=['spray_status'])

    def execute(self, userdata):
        global config
        curr_waypoint_idx = userdata.waypoint_status['curr_idx']
        spray_mode_list = config['spray_mode']
        spray_enable = False
        spray_continuous = False
        spot_turn_spray = False
        nozzle_num = []
        spray_direction = []

        if spray_mode_list is None or len(spray_mode_list) == 0:
            spray_enable = False

        else:

            while len(config['spray_mode']) != 0:
                ## Process like queue, FIFO
                spray_mode_list = config['spray_mode']
                spray_mode = spray_mode_list[0]
                rospy.logerr(spray_mode_list)
                nozzle_num = spray_mode['spray_params']['nozzle_num']
                spray_direction = spray_mode['spray_params']['spray_direction']

                ## Continuous Spraying
                if len(spray_mode['waypoint_idx']) > 1:
                    spot_turn_spray = False
                    start_idx = spray_mode['waypoint_idx'][0]
                    end_idx = spray_mode['waypoint_idx'][1]
                    if start_idx <= curr_waypoint_idx and curr_waypoint_idx < end_idx:
                        spray_enable = True
                        spray_continuous = True
                        break
                    else:
                        spray_enable = False
                        spray_continuous = False

                        # Spraying completed, pop from queue
                        if curr_waypoint_idx >= end_idx:
                            config['spray_mode'].pop(0)
                            rospy.loginfo("Spray Mode popped, left with: " + str(config['spray_mode']))
                        # Waiting for next path spray
                        elif curr_waypoint_idx < start_idx:
                            break

                # Short Spray
                elif len(spray_mode['waypoint_idx']) == 1:
                    spray_continuous = False
                    if curr_waypoint_idx == spray_mode['waypoint_idx'][0]:
                        spot_turn_spray = spray_mode['spray_params']['spot_turn_spray']
                        spray_enable = True
                        config['spray_mode'].pop(0)
                        break
                    # Already missed the waypoint
                    elif curr_waypoint_idx > spray_mode['waypoint_idx'][0]:
                        spray_enable = False
                        config['spray_mode'].pop(0)
                    else:
                        spray_enable = False
                        break


        userdata.spray_status['spray_enable'] = spray_enable
        userdata.spray_status['spray_continuous'] = spray_continuous
        userdata.spray_status['spot_turn_spray'] = spot_turn_spray
        userdata.spray_status['nozzle_num'] = nozzle_num 
        userdata.spray_status['spray_direction'] = spray_direction

        return 'loading_success'

class Waypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waypoint_completed', 'waypoint_give_up', 'failure'], input_keys=['waypoint_status'])

    def get_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.header.frame_id = GLOBAL_FRAME
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.position.z = waypoint[2]
        goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(*waypoint[3]))

        return goal

    def get_distance_from_goal(self, waypoint):
        global curr_pose_estimate_topic
        curr_pose_msg = rospy.wait_for_message(curr_pose_estimate_topic, PoseWithCovarianceStamped)
        diff_x = curr_pose_msg.pose.pose.position.x - waypoint[0]
        diff_y = curr_pose_msg.pose.pose.position.y - waypoint[1]

        return sqrt(diff_x ** 2 + diff_y ** 2)


    def execute(self, userdata):
        global feedback_pub

        rospy.loginfo("Next waypoint to go: " + str(userdata.waypoint_status))

        goal = self.get_goal(userdata.waypoint_status['waypoint'])

        move_base_client.send_goal(goal)

        # if the goal is not yet finished
        mission_state = 0
        was_paused = False
        go_origin = False

        while was_paused or not move_base_client.wait_for_result(timeout=rospy.Duration(0.5)):
            try:
                mission_state_msg = rospy.wait_for_message(MISSION_PLANNER_STATE_TOPIC, UInt8, timeout=rospy.Duration(0.4))
                mission_state = mission_state_msg.data

            except ROSException:
                pass
                # rospy.logerr("No mission_state received")

            # UInt8: 0 is Normal, 1 is Pause, 2 is Cancel and Stop, 3 is Cancel and go to origin
            if mission_state == 2 or mission_state  == 3:
                spray_msg = UInt8()
                spray_msg.data = 0
                spray_pub.publish(spray_msg)

                spray_direction_msg = UInt8()
                spray_direction_msg.data = 0
                spray_direction_pub.publish(spray_direction_msg)

            if mission_state == 0:
                if was_paused:
                    move_base_client.cancel_all_goals()
                    goal = self.get_goal(userdata.waypoint_status['waypoint'])
                    move_base_client.send_goal(goal)
                    was_paused = False
                    rospy.loginfo("Resuming Autonomous Waypoint")
                else:
                    # just resume as per normal until waypoint is reached
                    pass
            elif mission_state == 1:
                move_base_client.cancel_all_goals()
                was_paused = True
                rospy.logwarn("Autonomous Waypoint Pause")
            elif mission_state == 2:
                move_base_client.cancel_all_goals()
                rospy.logwarn("Autonomous Waypoint Cancel and Stop All Autonomous tasks")
                feedback_msg = Int8()
                feedback_msg.data = -1
                feedback_pub.publish(feedback_msg)
                return 'failure'
            elif mission_state == 3:
                # Dont repeatedly send goal to origin
                if not go_origin:
                    move_base_client.cancel_all_goals()
                    origin = config['waypoints']['origin'][0]
                    goal = self.get_goal(origin)
                    move_base_client.send_goal(goal)
                    rospy.logwarn("Autonomous Waypoint Cancel and Go to Origin")
                    go_origin = True
                    was_paused = False

        result = move_base_client.get_result()
        rospy.loginfo("[SDR_MISSION_PLANNER] Waypoint Result Achieved: {}".format(result))

        if go_origin:
            # Means end of entire waypointing task
            feedback_msg = Int8()
            feedback_msg.data = -1
            feedback_pub.publish(feedback_msg)
            return 'failure'
        else:
            dist = self.get_distance_from_goal(userdata.waypoint_status['waypoint'])

            # if too far from waypoint, considered give up
            if dist > 2.5:
                return 'waypoint_give_up'
            else:
                return 'waypoint_completed'

class Spray(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_spray', 'path_spray_success', 'short_spray_success'], input_keys=['spray_status'])

    def execute(self, userdata):
        spray_msg = UInt8()
        spray_msg.data = 0

        spray_direction_msg = UInt8()
        spray_direction_msg.data = 0

        if not userdata.spray_status['spray_enable']:
            spray_msg.data = 0
            spray_direction_msg.data = 0
            spray_direction_pub.publish(spray_direction_msg)
            spray_pub.publish(spray_msg)
            return 'no_spray'
      
        nozzle_num = userdata.spray_status['nozzle_num']
        if (len(nozzle_num) == 2):
            spray_msg.data = 3
        elif (len(nozzle_num) == 1 and nozzle_num[0] == 0):
            spray_msg.data = 1
        elif (len(nozzle_num) == 1 and nozzle_num[0] == 1):
            spray_msg.data = 2
        elif (len(nozzle_num) == 0):
            spray_msg.data = 0

        spray_direction = userdata.spray_status['spray_direction']
        if (spray_direction[0] == 0):
            spray_direction_msg.data = 0
        elif (spray_direction[0] == 1):
            spray_direction_msg.data = 1
        elif (spray_direction[0] == 2):
            spray_direction_msg.data = 2

        if userdata.spray_status['spray_continuous']:
            rospy.loginfo("[SDR_MISSION_PLANNER] Continuous Spray In Progress.....")
            spray_direction_pub.publish(spray_direction_msg)
            spray_pub.publish(spray_msg)
            return 'path_spray_success'
        else:
            if userdata.spray_status['spot_turn_spray']:
                rospy.loginfo("[SDR_MISSION_PLANNER] Spot Turn Spray In Progress.....")
                spray_direction_msg.data = 0
                spray_msg.data = 5
                spray_direction_pub.publish(spray_direction_msg)
                spray_pub.publish(spray_msg)
                rospy.sleep(20.0)
                rospy.loginfo("[SDR_MISSION_PLANNER] Spot Turn Spray Completed")
            else:
                rospy.loginfo("[SDR_MISSION_PLANNER] Short Burst Spray In Progress.....")
                spray_direction_msg.data = 0
                spray_direction_pub.publish(spray_direction_msg)
                spray_pub.publish(spray_msg)
                rospy.sleep(4.0)
                spray_msg.data = 0
                spray_pub.publish(spray_msg)
                rospy.loginfo("[SDR_MISSION_PLANNER] Short Burst Spray Completed.....")

            return 'short_spray_success'
        



## MAIN TASK ##

class AutonomousWaypointTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):

        ######## LOADING SUB STATE MACHINE TO LOAD MAPS AND WAYPOINT, SPRAY CONFIGS #################
        sm_load_params = smach.StateMachine(outcomes=['loading_success', 'loading_failure'])

        with sm_load_params:
            smach.StateMachine.add('Load_Navigation', LoadNavigation(),
                                   transitions={'success': 'Load_Waypoints',
                                                'failure': 'loading_failure'})

            smach.StateMachine.add('Load_Waypoints', LoadWaypoints(),
                                   transitions={'success': 'loading_success',
                                                'failure': 'loading_failure'})
            
            # smach.StateMachine.add('Load_Init_Pose', LoadInitPose(),
            #                        transitions={'success': 'loading_success',
            #                                     'failure': 'loading_failure'})

        #############################################################################################

        sm = smach.StateMachine(outcomes=['success', 'failure'])

        with sm:
            sm.userdata.spray_status = {'spray_enable': False, 'spray_continuous': False, 'spot_turn_spray': False, 'nozzle_num': []}
            sm.userdata.waypoint_status = dict()

            smach.StateMachine.add('Load_Params', sm_load_params, 
                                    transitions={'loading_success': 'Get_Next_Waypoint_State',
                                                 'loading_failure': 'failure'})

            smach.StateMachine.add('Get_Next_Waypoint_State', GetNextWaypointState(), 
                                    transitions={'loading_success': 'Waypoint_State',
                                                 'mission_finished': 'success',
                                                 'loading_failure': 'failure'},
                                    remapping={'waypoint_status':'waypoint_status'})

            smach.StateMachine.add('Waypoint_State', Waypoint(), 
                                    transitions={'waypoint_completed': 'Get_Next_Spray_State',
                                                 'waypoint_give_up': 'Get_Next_Waypoint_State',
                                                 'failure': 'failure'})


            smach.StateMachine.add('Get_Next_Spray_State', GetNextSprayState(), 
                                    transitions={'loading_success': 'Spray_State',
                                                 'loading_failure': 'failure'},
                                    remapping={'spray_status':'spray_status'})

            smach.StateMachine.add('Spray_State', Spray(), 
                                    transitions={'no_spray': 'Get_Next_Waypoint_State',
                                                 'short_spray_success': 'Get_Next_Spray_State',
                                                 'path_spray_success': 'Get_Next_Waypoint_State'})

                        ##########################################################################


            # smach.StateMachine.add('Robot_Stuck', CryForHelp(), 
            #                         transitions={'recovered': 'Waypoint_And_Spray_Concurrent_Task'
            #                                      'return_home': 'Go_Home'
            #                                      'cancel': 'task_failure'
            #                                      'waiting': 'Robot_Stuck'})

            


        result = sm.execute()
        return result

def main():
    rospy.init_node('Autonomous_Navigation_Task', anonymous=True)
    load_rosparams()
    
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    with sm:
        smach.StateMachine.add('Autonomous_Waypoint_Task', AutonomousWaypointTask(), 
                               transitions={'success':'success', 'failure':'failure'})

    outcome = sm.execute()
    return outcome


if __name__ == '__main__':
    main()
