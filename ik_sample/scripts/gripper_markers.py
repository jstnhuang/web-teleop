#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('interactive_markers')
roslib.load_manifest('tf')

import numpy
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from ik import IK
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction



class GripperMarkers:

    _offset = 0.00
    def __init__(self, side_prefix):
        self._ik_service = IK(side_prefix)

        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()

        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()

        self.r_joint_names = ['r_shoulder_pan_joint',
                              'r_shoulder_lift_joint',
                              'r_upper_arm_roll_joint',
                              'r_elbow_flex_joint',
                              'r_forearm_roll_joint',
                              'r_wrist_flex_joint',
                              'r_wrist_roll_joint']
        self.l_joint_names = ['l_shoulder_pan_joint',
                              'l_shoulder_lift_joint',
                              'l_upper_arm_roll_joint',
                              'l_elbow_flex_joint',
                              'l_forearm_roll_joint',
                              'l_wrist_flex_joint',
                              'l_wrist_roll_joint']

	
	self.side_prefix = side_prefix
        self._im_server = InteractiveMarkerServer("ik_request_markers_{}".format(side_prefix))
	self._tf_listener = TransformListener()
        self._menu_handler = MenuHandler()

        self._menu_handler.insert("Move {} arm here".format(side_prefix), callback=self.move_to_pose_cb)

        self.is_control_visible = True
        self.ee_pose = self.get_ee_pose()
        self.update_viz()
        self._menu_handler.apply(self._im_server, 'ik_target_marker')
        self._im_server.applyChanges()

    def get_ee_pose(self):
	from_frame = 'base_link'
	to_frame = self.side_prefix + '_wrist_roll_link'
	try:
	    t = self._tf_listener.getLatestCommonTime(from_frame, to_frame)
	    (pos, rot) = self._tf_listener.lookupTransform(from_frame, to_frame, t)
	except Exception as e:
	    rospy.logerr('Could not get end effector pose through TF.')
	    pos = [1.0, 0.0, 1.0]
	    rot = [0.0, 0.0, 0.0, 1.0]
	    import traceback
 	    traceback.print_exc()


	return Pose(Point(pos[0], pos[1], pos[2]),
			Quaternion(rot[0], rot[1], rot[2], rot[3]))

    def move_to_pose_cb(self, dummy):
        ik_sol = self._ik_service.get_ik_for_ee(self.ee_pose)
        self.move_to_joints(self.side_prefix, [ik_sol], 1.0)

    def marker_clicked_cb(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.ee_pose = feedback.pose
            self.update_viz()
            self._menu_handler.reApply(self._im_server)
            self._im_server.applyChanges()

        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            pass
#            rospy.loginfo('Changing visibility of the pose controls to {}'.format(not self.is_control_visible))
#            if (self.is_control_visible):
#                self.is_control_visible = False
#            else:
#                self.is_control_visible = True
        else:
            rospy.loginfo('Unhandled event: ' + str(feedback.event_type))

    def update_viz(self):
        
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = 'base_link'
        pose = self.ee_pose
        
        menu_control = self._add_gripper_marker(menu_control)
#        text_pos = Point()
#        text_pos.x = pose.position.x
#        text_pos.y = pose.position.y
#        text_pos.z = pose.position.z + 0.1
#        text = "x=%.3f y=%.3f z=%.3f" % (pose.position.x, pose.position.y, pose.position.z)
#        menu_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
#                                           id=0, scale=Vector3(0, 0, 0.03),
#                                           text=text,
#                                           color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
#                                           header=Header(frame_id=frame_id),
#                                           pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))
#        label_pos = Point()
#        label_pos.x = pose.position.x
#        label_pos.y = pose.position.y
#        label_pos.z = pose.position.z
#        label = "{} arm".format(self.side_prefix)
#        menu_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
#                                           id=0, scale=Vector3(0, 0, 0.03),
#                                           text=label,
#                                           color=ColorRGBA(0.0, 0.0, 0.0, 0.9),
#                                           header=Header(frame_id=frame_id),
#                                           pose=Pose(label_pos, Quaternion(0, 0, 0, 1))))

        int_marker = InteractiveMarker()
        int_marker.name = 'ik_target_marker'
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose
        int_marker.scale = 0.2
        self._add_6dof_marker(int_marker)
        int_marker.controls.append(menu_control)
        self._im_server.insert(int_marker, self.marker_clicked_cb)

    def _add_gripper_marker(self, control):
        is_hand_open=False
        if is_hand_open:
            angle = 28 * numpy.pi / 180.0
        else:
            angle = 0
        transform1 = tf.transformations.euler_matrix(0, 0, angle)
        transform1[:3, 3] = [0.07691 - GripperMarkers._offset, 0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(transform1, transform2)
        mesh1 = self._make_mesh_marker()
        mesh1.mesh_resource = ('package://pr2_description/meshes/gripper_v0/gripper_palm.dae')
        mesh1.pose.position.x = -GripperMarkers._offset
        mesh1.pose.orientation.w = 1
        mesh2 = self._make_mesh_marker()
        mesh2.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger.dae')
        mesh2.pose = GripperMarkers.get_pose_from_transform(t_proximal)
        mesh3 = self._make_mesh_marker()
        mesh3.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger_tip.dae')
        mesh3.pose = GripperMarkers.get_pose_from_transform(t_distal)
        quat = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_from_euler(numpy.pi, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, angle))
        transform1 = tf.transformations.quaternion_matrix(quat)
        transform1[:3, 3] = [0.07691 - GripperMarkers._offset, -0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(transform1,transform2)
        mesh4 = self._make_mesh_marker()
        mesh4.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger.dae')
        mesh4.pose = GripperMarkers.get_pose_from_transform(t_proximal)
        mesh5 = self._make_mesh_marker()
        mesh5.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger_tip.dae')
        mesh5.pose = GripperMarkers.get_pose_from_transform(t_distal)

        control.markers.append(mesh1)
        control.markers.append(mesh2)
        control.markers.append(mesh3)
        control.markers.append(mesh4)
        control.markers.append(mesh5)
        return control

    @staticmethod
    def get_pose_from_transform(transform):
	pos = transform[:3,3].copy()
	rot = tf.transformations.quaternion_from_matrix(transform)
	return Pose(Point(pos[0], pos[1], pos[2]),
			Quaternion(rot[0], rot[1], rot[2], rot[3]))

    def _make_mesh_marker(self):
        mesh = Marker()
        mesh.mesh_use_embedded_materials = False
        mesh.type = Marker.MESH_RESOURCE
    	mesh.scale.x = 1.0
        mesh.scale.y = 1.0
        mesh.scale.z = 1.0

        ik_sol = self._ik_service.get_ik_for_ee(self.ee_pose)
        if ik_sol == None:
            mesh.color = ColorRGBA(1.0, 0.0, 0.0, 0.6)
        else:
            mesh.color = ColorRGBA(0.0, 1.0, 0.0, 0.6)
        return mesh

    def _add_6dof_marker(self, int_marker):
        is_fixed = True
        control = self._make_6dof_control('rotate_x', Quaternion(1, 0, 0, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_x', Quaternion(1, 0, 0, 1), True, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('rotate_z', Quaternion(0, 1, 0, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_z', Quaternion(0, 1, 0, 1), True, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('rotate_y', Quaternion(0, 0, 1, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_y', Quaternion(0, 0, 1, 1), True, is_fixed)
        int_marker.controls.append(control)

    def _make_6dof_control(self, name, orientation, is_move, is_fixed):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation = orientation
        control.always_visible = True
        if (self.is_control_visible):
            if is_move:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if is_fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        return control

    def move_to_joints(self, side_prefix, positions, time_to_joint):
        '''Moves the arm to the desired joints'''
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        time_move = time_to_joint
        for pose in positions:
            velocities = [0] * len(pose)
            traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=pose,
                            velocities=velocities, time_from_start=rospy.Duration(time_move)))
            time_move += time_to_joint

        if (side_prefix == 'r'):
            traj_goal.trajectory.joint_names = self.r_joint_names
            self.r_traj_action_client.send_goal(traj_goal)
        else:
            traj_goal.trajectory.joint_names = self.l_joint_names
            self.l_traj_action_client.send_goal(traj_goal)

if __name__=='__main__':
    rospy.init_node('ik_target_marker_server')
    gm = GripperMarkers('r')
    gm = GripperMarkers('l')
    rospy.spin()


