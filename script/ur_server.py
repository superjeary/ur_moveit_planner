import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    group = moveit_commander.MoveGroupCommander("manipulator")

    while(rospy.is_shutdown()):
        current_joint_values = group.get_current_joint_values()
        print "============ Joint values: ", current_joint_values
        
        current_pose = group.get_current_pose().pose
        print "============ pose: ", current_pose



if __name__=='__main__':
  try:
      move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
      pass