(function(driverUi) {
  var viewerId = 'mjpeg';

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

  driverUi.init = function() {
    var ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });
    
    var teleop = new KEYBOARDTELEOP.Teleop({
      ros: ros,
      baseTopic: '/base_controller/command'
    });
  
    var viewerDimensions = getViewerDimensions();
    var viewer = new MJPEGCANVAS.MultiStreamViewer({
      divID : viewerId,
      host : 'localhost',
      width : viewerDimensions.width,
      height : viewerDimensions.height,
      topics : ['/wide_stereo/left/image_color'],
      labels : ['Robot View']
    });
  
    $(window).resize(handleResize);
  }
} (window.driverUi=window.driverUi || {}));
