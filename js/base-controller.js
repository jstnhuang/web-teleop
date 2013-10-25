(function(baseController) {
  var cmdVel;

  function genTwist(ax, ay, az, lx, ly, lz) {
    return new ROSLIB.Message({
      angular: {
        x: ax,
        y: ay,
        z: az
      },
      linear: {
        x: lx,
        y: ly,
        z: lz
      }
    });
  }

  baseController.init = function(ros, topic) {
    cmdVel = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: 'geometry_msgs/Twist'
    });
  }

  baseController.stop = function() {
    var twist = genTwist(0, 0, 0, 0, 0, 0);
    cmdVel.publish(twist);
  }

  baseController.moveForward = function(speed) {
    var twist = genTwist(0, 0, 0, speed, 0, 0);
    cmdVel.publish(twist);
  }

  baseController.moveBackward = function(speed) {
    var twist = genTwist(0, 0, 0, -speed, 0, 0);
    cmdVel.publish(twist);
  }

  baseController.moveLeft = function(speed) {
    var twist = genTwist(0, 0, 0, 0, speed, 0);
    cmdVel.publish(twist);
  }

  baseController.moveRight = function(speed) {
    var twist = genTwist(0, 0, 0, 0, -speed, 0);
    cmdVel.publish(twist);
  }

  baseController.rotateClockwise = function(speed) {
    var twist = genTwist(0, 0, -speed, 0, 0, -speed);
    cmdVel.publish(twist);
  }

  baseController.rotateCounterClockwise = function(speed) {
    var twist = genTwist(0, 0, speed, 0, 0, speed);
    cmdVel.publish(twist);
  }
} (window.baseController=window.baseController || {}));
