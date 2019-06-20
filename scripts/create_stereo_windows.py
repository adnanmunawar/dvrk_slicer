# !/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019
#     (www.aimlab.wpi.edu)

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <http://www.aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   0.1
# */
# //==============================================================================
import slicer


# Load the scene first
slicer.util.loadScene('/home/adnan/NewWorkingHamlynDemo/AnatomicalMRIRegisteredToDTI.mrml')

# Create a Widget for Right Window
#
#
layout_name_right = "Right_Window"
layout_label = "T3"
# Create MRML node
view_node_right = slicer.vtkMRMLViewNode()
view_node_right.SetName(layout_name_right)
view_node_right.SetLayoutName(layout_name_right)
view_node_right.SetLayoutLabel(layout_label)
view_node_right = slicer.mrmlScene.AddNode(view_node_right)
view_node_right.SetName(layout_name_right)

# Create widget
view_widget_right = slicer.qMRMLThreeDWidget()
view_widget_right.setMRMLScene(slicer.mrmlScene)
view_widget_right.setMRMLViewNode(view_node_right)
view_widget_right.setWindowTitle(layout_name_right) # Make sure to name the window
view_widget_right.show()


# Create a Widget for Right Window
#
#
layoutNameLeft = "Left_Window"
# Create MRML node
view_node_left = slicer.vtkMRMLViewNode()
view_node_left.SetName(layoutNameLeft)
view_node_left.SetLayoutName(layoutNameLeft)
view_node_left.SetLayoutLabel(layout_label)
view_node_left = slicer.mrmlScene.AddNode(view_node_left)
view_node_left.SetName(layoutNameLeft)

# Create widget
view_widget_left = slicer.qMRMLThreeDWidget()
view_widget_left.setMRMLScene(slicer.mrmlScene)
view_widget_left.setMRMLViewNode(view_node_left)
view_widget_left.setWindowTitle(layoutNameLeft) # Make sure to name the window
view_widget_left.show()

# Create a Transform for the Camera
camera_transform = slicer.mrmlScene.CreateNodeByClass('vtkMRMLLinearTransformNode')
camera_transform.SetName('CameraTransform')
slicer.mrmlScene.AddNode(camera_transform)

# Get Right camera handle
camera_right = slicer.modules.cameras.logic().GetViewActiveCameraNode(view_node_right)
# camera_right.GetCamera().Azimuth(1.5)
camera_right.SetName('CameraRight')
camera_right.SetAndObserveTransformNodeID(camera_transform.GetID()) # Assign Transform for the Camera
camera_right.GetCamera().SetPosition((43, 498, 0))
camera_right.GetCamera().SetRoll(-90)
# Get Left camera handle
camera_left = slicer.modules.cameras.logic().GetViewActiveCameraNode(view_node_left)
# camera_left.GetCamera().Azimuth(-1.5)
camera_left.SetName('CameraLeft')
camera_left.SetAndObserveTransformNodeID(camera_transform.GetID()) # Assign Transform for the Camera
camera_left.GetCamera().SetPosition((-43, 498, 0))
camera_left.GetCamera().SetRoll(90)

print 'Creating OpenIGTLinkIF connector'

igtl_connector = slicer.vtkMRMLIGTLConnectorNode()
slicer.mrmlScene.AddNode(igtl_connector)

igtl_connector.SetName('DVRK_IGTL_CONNECTOR')
igtl_connector.SetTypeClient('localhost', 11344)
igtl_connector.Start()
igtl_connector.RegisterOutgoingMRMLNode(camera_transform)
