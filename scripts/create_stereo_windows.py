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
