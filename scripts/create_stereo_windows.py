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

layoutNameRight = "RightView"
layoutLabel = "T3"

# Create MRML node
viewNodeRight = slicer.vtkMRMLViewNode()
viewNodeRight.SetName(layoutNameRight)
viewNodeRight.SetLayoutName(layoutNameRight)
viewNodeRight.SetLayoutLabel(layoutLabel)
viewNodeRight = slicer.mrmlScene.AddNode(viewNodeRight)
viewNodeRight.SetName('RightView')

# Create widget
viewWidgetRight = slicer.qMRMLThreeDWidget()

viewWidgetRight.setMRMLScene(slicer.mrmlScene)
viewWidgetRight.setMRMLViewNode(viewNodeRight)
viewWidgetRight.show()

layoutNameLeft = "LeftView"

# Create MRML node
viewNodeLeft = slicer.vtkMRMLViewNode()
viewNodeLeft.SetName(layoutNameLeft)
viewNodeLeft.SetLayoutName(layoutNameLeft)
viewNodeLeft.SetLayoutLabel(layoutLabel)
viewNodeLeft = slicer.mrmlScene.AddNode(viewNodeLeft)
viewNodeLeft.SetName('LeftView')

# Create widget
viewWidgetLeft = slicer.qMRMLThreeDWidget()

viewWidgetLeft.setMRMLScene(slicer.mrmlScene)
viewWidgetLeft.setMRMLViewNode(viewNodeLeft)
viewWidgetLeft.show()


camera_right = slicer.modules.cameras.logic().GetViewActiveCameraNode(viewNodeRight)
camera_right.GetCamera().Azimuth(-5)
camera_left = slicer.modules.cameras.logic().GetViewActiveCameraNode(viewNodeLeft)
camera_left.GetCamera().Azimuth(5)

camera_right.SetName('CameraRight')
camera_left.SetName('CameraLeft')

cameraTransform = slicer.mrmlScene.CreateNodeByClass('vtkMRMLLinearTransformNode')

cameraTransform.SetName('CameraTransform')

slicer.mrmlScene.AddNode(cameraTransform)

camera_right.SetAndObserveTransformNodeID(cameraTransform.GetID())
camera_left.SetAndObserveTransformNodeID(cameraTransform.GetID())

print 'Creating OpenIGTLinkIF connector'

igtlConnector = slicer.vtkMRMLIGTLConnectorNode()
slicer.mrmlScene.AddNode(igtlConnector)

igtlConnector.SetName('DVRK_IGTL_CONNECTOR')
igtlConnector.SetTypeClient('localhost', 11344)

igtlConnector.Start()
igtlConnector.RegisterOutgoingMRMLNode(cameraTransform)