#ifndef _ROS_pr2_controllers_msgs_JointTrajectoryAction_h
#define _ROS_pr2_controllers_msgs_JointTrajectoryAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_controllers_msgs/JointTrajectoryActionGoal.h"
#include "pr2_controllers_msgs/JointTrajectoryActionResult.h"
#include "pr2_controllers_msgs/JointTrajectoryActionFeedback.h"

namespace pr2_controllers_msgs
{

  class JointTrajectoryAction : public ros::Msg
  {
    public:
      pr2_controllers_msgs::JointTrajectoryActionGoal action_goal;
      pr2_controllers_msgs::JointTrajectoryActionResult action_result;
      pr2_controllers_msgs::JointTrajectoryActionFeedback action_feedback;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pr2_controllers_msgs/JointTrajectoryAction"; };
    const char * getMD5(){ return "a04ba3ee8f6a2d0985a6aeaf23d9d7ad"; };

  };

}
#endif