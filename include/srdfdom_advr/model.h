/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author Ioan Sucan */

#ifndef SRDF_ADVR_MODEL_
#define SRDF_ADVR_MODEL_

#include <srdfdom/model.h>

// NOTE custom definition
namespace urdf
{
typedef boost::shared_ptr<const ::urdf::Link> LinkConstSharedPtr;
typedef boost::shared_ptr<const ::urdf::Joint> JointConstSharedPtr;
}

/// Main namespace
namespace srdf_advr
{

/** \brief Representation of semantic information about the robot */
class Model : public srdf::Model
{
public:

  Model():
    srdf::Model()
  {
  }

  ~Model()
  {
  }

  /// \brief Load Model from TiXMLElement
  bool initXml(const urdf::ModelInterface &urdf_model, TiXmlElement *xml);

  // Some joints can be disabled. This structure specifies information about such joints
  struct DisabledJoint
  {
      /// The name of the disabled joint
      std::string name_;
  };


  /// Get the list of known disabled joints
  const std::vector<DisabledJoint>& getDisabledJoints() const
  {
    return disabled_joints_;
  }

private:

  void loadDisabledJoints(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml);

  std::vector<DisabledJoint>     disabled_joints_;

};

}
#endif
