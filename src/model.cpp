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

#include <srdfdom_advr/model.h>
#include <console_bridge/console.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <set>
#include <limits>


void srdf_advr::Model::loadDisabledJoints(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml)
{

    for (TiXmlElement* c_xml = robot_xml->FirstChildElement("disabled_joint"); c_xml; c_xml = c_xml->NextSiblingElement("disabled_joint"))
    {
        const char *name = c_xml->Attribute("name");
        if (!name)
        {
            logError("No name specified for disabled joint. Ignoring.");
            continue;
        }
        DisabledJoint dj;
        dj.name_ = boost::trim_copy(std::string(name));

        disabled_joints_.push_back(dj);
    }
}

bool srdf_advr::Model::initXml(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml)
{
  srdf::Model::initXml(urdf_model, robot_xml);
  disabled_joints_.clear();
  loadDisabledJoints(urdf_model, robot_xml);

  return true;
}

