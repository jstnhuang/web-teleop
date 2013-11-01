(function(interactView) {
  var viewerId = 'viewer';

  function getViewerDimensions() {
    if (window.innerHeight * 4 / 3 <= window.innerWidth) {
      return {
        width: window.innerHeight * 4 / 3,
        height: window.innerHeight
      };
    } else {
      return {
        width: window.innerWidth,
        height: window.innerWidth * 3 / 4
      };
    }
  }

  function handleResize(resizeEvent) {
    var viewerDimensions = getViewerDimensions();
    var viewerCanvas = $('#' + viewerId).children().first();
    viewerCanvas.width(viewerDimensions.width);
    viewerCanvas.height(viewerDimensions.height);
  }

  interactView.init = function(options) {
    var options = options || {};
    viewerId = options.viewerId || 'viewer';

    var ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    var teleop = new KEYBOARDTELEOP.Teleop({
      ros: ros,
      baseTopic: '/base_controller/command'
    });

    var viewerDimensions = getViewerDimensions();
    var viewer = new ROS3D.Viewer({
      divID: viewerId,
      width: viewerDimensions.width,
      height: viewerDimensions.height,
      antialias: true
    });
    viewer.addObject(new ROS3D.Grid());
  
    var tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: '/base_link'
    });

    var urdfClient = new ROS3D.UrdfClient({
      ros: ros,
      tfClient: tfClient,
      path: 'http://resources.robotwebtools.org/',
      rootObject: viewer.scene
    });

    var imClient = new ROS3D.InteractiveMarkerClient({
      ros: ros,
      tfClient: tfClient,
      topic: '/ik_request_markers_l',
      camera: viewer.camera,
      rootObject: viewer.selectableObjects
    });

    depthCloud = new ROS3D.DepthCloud({
      url: '/streams/depthcloud_encoded.webm',
      f: 525.0
    });
    depthCloud.startStream();

    var kinectNode = new ROS3D.SceneNode({
      frameID: '/head_mount_kinect_rgb_optical_frame',
      tfClient: tfClient,
      object: depthCloud
    });
    viewer.scene.add(kinectNode);
  
    $(window).resize(handleResize);
  }
} (window.interactView=window.interactView || {}));
