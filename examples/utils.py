import rospy
from ipywidgets import Button, Layout, GridBox, VBox, Box, FloatSlider, Checkbox, interact
from giskardpy.python_interface.python_interface import GiskardWrapper
from giskardpy.goals.joint_goals import JointPositionList
from giskardpy.monitors.joint_monitors import JointGoalReached
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, QuaternionStamped
import roslib; roslib.load_manifest('urdfdom_py')
from rqt_joint_trajectory_controller import joint_limits_urdf

rospy.init_node('giskard_blockly_playground')
# Giskard wrapper instance
gk_wrapper = GiskardWrapper()
CMD_VEL_TOPIC = '/cmd_vel'
ROBOT_DESCRIPTION = '/robot_description'

# moving motion
def add_cartesian_position(pos, root_link='map', tip_link='base_link'):
    pos_stamp = PointStamped()
    pos_stamp.header.frame_id = root_link
    pos_stamp.point = pos
    gk_wrapper.add_default_end_motion_conditions()
    gk_wrapper.motion_goals.add_cartesian_position(
        root_link=root_link,
        tip_link=tip_link,
        goal_point=pos_stamp,
    )
    gk_wrapper.execute()

# rotation motion
def add_cartesian_orientation(ori, root_link='map', tip_link='base_link'):
    ori_stamp = QuaternionStamped()
    ori_stamp.header.frame_id = root_link
    ori_stamp.quaternion = ori
    gk_wrapper.add_default_end_motion_conditions()
    gk_wrapper.motion_goals.add_cartesian_orientation(
        root_link=root_link,
        tip_link=tip_link,
        goal_orientation=ori_stamp,
    )
    gk_wrapper.execute()

# joint position motion
def add_joint_position(joint_goal_list):
    joint_goal = {key: value for d in joint_goal_list for key, value in d.items()}
    gk_wrapper.motion_goals.add_joint_position(goal_state=joint_goal)
    gk_wrapper.add_default_end_motion_conditions()
    gk_wrapper.execute()

# add_cartesian_pose
def add_cartesian_pose(pos, ori, root_link='map', tip_link='base_link'):
    if pos is None and ori is None:
        print("Both position and orientation are None!!!")
        return
    if pos is None:
        add_cartesian_orientation(ori, root_link=root_link, tip_link=tip_link)
        return
    if ori is None:
        add_cartesian_position(pos, root_link=root_link, tip_link=tip_link)
        return
    pose_stamp = PoseStamped()
    pose_stamp.header.frame_id = root_link
    pose_stamp.pose.position = pos
    pose_stamp.pose.orientation = ori
    gk_wrapper.add_default_end_motion_conditions()
    gk_wrapper.motion_goals.add_cartesian_pose(
        root_link=root_link,
        tip_link=tip_link,
        goal_pose=pose_stamp,
    )
    gk_wrapper.execute()

# get robot links
def get_links():
    return gk_wrapper.world.get_group_info(gk_wrapper.world.get_group_names()[0]).links

# get all robot links
def get_controlled_joints():
    return gk_wrapper.world.get_controlled_joints(gk_wrapper.world.get_group_names()[0])

# get joint state
def get_joint_state():
    return gk_wrapper.world.get_group_info(gk_wrapper.world.get_group_names()[0]).joint_state

# Contorl robot base by cmd_vel message
def cmd_vel_move(speed, time):
    cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=100)
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = speed
    cmd_vel_pub.publish(cmd_vel_msg)
    rospy.sleep(time)
    cmd_vel_msg.linear.x = 0
    cmd_vel_pub.publish(cmd_vel_msg)

def cmd_vel_turn(speed, time):
    cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=100)
    cmd_vel_msg = Twist()
    cmd_vel_msg.angular.z = speed
    cmd_vel_pub.publish(cmd_vel_msg)
    rospy.sleep(time)
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)

# Diplay joint trajectory controller
def display_joint_trajectory_controller():
    cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=100)
    cmd_vel_msg = Twist()
    # robot_steering, similar to "rqt_robot_steering"
    linear_x = FloatSlider(
        value=0,
        min=-1,
        max=1,
        step=0.1,
        description='Moving',
        orientation='vertical',
        readout=True,
        readout_format='.1f',
    )

    def on_linear_x_change(v):
        cmd_vel_msg.linear.x = v
        cmd_vel_pub.publish(cmd_vel_msg)

    linear_x.observe(lambda v: on_linear_x_change(v['new']), names='value')

    # slider for rotation velocity
    angular_z = FloatSlider(
        value=0,
        min=-1,
        max=1,
        step=0.1,
        description='Rotation',
        readout=True,
        readout_format='.1f',
    )

    def on_angular_z_change(v):
        cmd_vel_msg.angular.z = -v
        cmd_vel_pub.publish(cmd_vel_msg)

    angular_z.observe(lambda v: on_angular_z_change(v['new']), names='value')

    def generate_slider(config):
        slider = FloatSlider(
            value=config['value'],
            min=config['min'],
            max=config['max'],
            step=(config['max'] - config['min']) / 20,
            continuous_update=False,
            description=config['name'],
            readout=True,
            layout=Layout(width='80%'),
            style=dict(
                description_width='10rem'
            ),
            readout_format='.3f',
        )
        slider.observe(lambda v: add_joint_position([{config['name']: v['new']}]), names='value')
        return slider

    def format_slider_config(j_name):
        if j_name in joint_limits:
            config = {}
            config['name'] = j_name
            config['min'] = joint_limits[j_name]['min_position']
            config['max'] = joint_limits[j_name]['max_position']
            config['value'] = joint_state.position[[i.split('/')[1] for i in joint_state.name].index(j_name)]
            return config
        else:
            # print(f"Joint '{j_name}' not found in URDF!")
            return None
    # Display Sliders
    joint_limits = joint_limits_urdf.get_joint_limits(key=ROBOT_DESCRIPTION)
    joint_state = get_joint_state()
    joint_slider_config = [format_slider_config(i) for i in get_controlled_joints()]
    joint_slider_config = [x for x in joint_slider_config if x is not None]
    joint_slider_config = sorted(joint_slider_config, key=lambda x: x['name'])
    joint_slider_widgets = [generate_slider(i) for i in joint_slider_config]
    display(Box([linear_x, angular_z]))
    display(VBox(joint_slider_widgets))
