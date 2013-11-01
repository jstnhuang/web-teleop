var KEYBOARDTELEOP = KEYBOARDTELEOP || {
  REVISION : '2'
};

KEYBOARDTELEOP.Teleop = function(options) {
  options = options || {};
  var ros = options.ros;
  var baseTopic = options.baseTopic || '/cmd_vel';
  var speed = options.speed || 1

  baseModel.init(ros, baseTopic);
  headModel.init(ros);
 
  function handleKeyDown(keyDownEvent) {
    var baseSpeed = 1;
    var headSpeed = 0.01;
    var baseBindings = {
      'W': baseModel.moveForward,
      'A': baseModel.moveLeft,
      'S': baseModel.moveBackward,
      'D': baseModel.moveRight,
      'Q': baseModel.rotateCounterClockwise,
      'E': baseModel.rotateClockwise,
    }
    var headBindings = {
      'I': headModel.lookUp,
      'J': headModel.lookLeft,
      'K': headModel.lookDown,
      'L': headModel.lookRight
    }
    var key = String.fromCharCode(keyDownEvent.keyCode);
    if (baseBindings.hasOwnProperty(key)) {
      baseBindings[key](baseSpeed);
    } else if (headBindings.hasOwnProperty(key)) {
      headBindings[key](headSpeed);
    } else {
    }
  }

  function handleKeyUp(keyUpEvent) {
    baseModel.stop();
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
