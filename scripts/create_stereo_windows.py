viewWidgets = []
views = []
cameras = []
layoutManager = slicer.app.layoutManager()
for threeDViewIndex in range(layoutManager.threeDViewCount) :
  viewWidget = layoutManager.threeDWidget(threeDViewIndex)
  view = viewWidget.threeDView()
  threeDViewNode = view.mrmlViewNode()
  cameraNode = slicer.modules.cameras.logic().GetViewActiveCameraNode(threeDViewNode)
  cameraNode.SetName('Camera' + str(threeDViewIndex))
  viewWidgets.append(viewWidget)
  views.append(threeDViewNode)
  cameras.append(cameraNode)

vwr = viewWidgets[0]
vwl = viewWidgets[2]

vwr.setParent(None)
vwl.setParent(None)

vwr.show()
vwl.show()

camera_right = cameras[0]
camera_left = cameras[2]

camera_right.SetName('CameraRight')
camera_left.SetName('CameraLeft')

tr_right = slicer.mrmlScene.CreateNodeByClass('vtkMRMLTransformNode')
tr_left = slicer.mrmlScene.CreateNodeByClass('vtkMRMLTransformNode')

tr_right.SetName('StereoRight')
tr_left.SetName('StereoLeft')

slicer.mrmlScene.AddNode(tr_right)
slicer.mrmlScene.AddNode(tr_left)

camera_right.SetAndObserveTransformNodeID(tr_right.GetID())
camera_left.SetAndObserveTransformNodeID(tr_left.GetID())
