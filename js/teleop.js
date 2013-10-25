var KEYBOARDTELEOP = KEYBOARDTELEOP || {
  REVISION : '2'
};

KEYBOARDTELEOP.Teleop = function(options) {
  options = options || {};
  var ros = options.ros;
  var baseTopic = options.baseTopic || '/cmd_vel';
  var viewer = '#' + (options.viewerId || 'mjpeg')
  var speed = options.speed || 1
  var that = this;

  baseController.init(ros, baseTopic);
  headController.init(ros);
 
  function handleKeyDown(keyDownEvent) {
    var baseSpeed = 1;
    var headSpeed = 0.01;
    var baseBindings = {
      'W': baseController.moveForward,
      'A': baseController.moveLeft,
      'S': baseController.moveBackward,
      'D': baseController.moveRight,
      'Q': baseController.rotateCounterClockwise,
      'E': baseController.rotateClockwise,
    }
    var headBindings = {
      'I': headController.lookUp,
      'J': headController.lookLeft,
      'K': headController.lookDown,
      'L': headController.lookRight
    }
    var key = String.fromCharCode(keyDownEvent.keyCode);
    if (baseBindings.hasOwnProperty(key)) {
      baseBindings[key](baseSpeed);
    } else {
      headBindings[key](headSpeed);
    }
  }

  function handleKeyUp(keyUpEvent) {
    baseController.stop();
  }

  var body = $('body');
  body.keydown(function(keyDownEvent) {
    handleKeyDown(keyDownEvent);
  });
  body.keyup(function(keyUpEvent) {
    handleKeyUp(keyUpEvent);
  });
};

KEYBOARDTELEOP.Teleop.prototype.__proto__ = EventEmitter2.prototype;
