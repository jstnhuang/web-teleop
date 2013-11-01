(function(baseModel) {
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

  baseModel.init = function(ros, topic) {
    cmdVel = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: 'geometry_msgs/Twist'
    });
  }

  baseModel.stop = function() {
    var twist = genTwist(0, 0, 0, 0, 0, 0);
    cmdVel.publish(twist);
  }

  baseModel.moveForward = function(speed) {
    var twist = genTwist(0, 0, 0, speed, 0, 0);
    cmdVel.publish(twist);
  }

  baseModel.moveBackward = function(speed) {
    var twist = genTwist(0, 0, 0, -speed, 0, 0);
    cmdVel.publish(twist);
  }

  baseModel.moveLeft = function(speed) {
    var twist = genTwist(0, 0, 0, 0, speed, 0);
    cmdVel.publish(twist);
  }

  baseModel.moveRight = function(speed) {
    var twist = genTwist(0, 0, 0, 0, -speed, 0);
    cmdVel.publish(twist);
  }

  baseModel.rotateClockwise = function(speed) {
    var twist = genTwist(0, 0, -speed, 0, 0, -speed);
    cmdVel.publish(twist);
  }

  baseModel.rotateCounterClockwise = function(speed) {
    var twist = genTwist(0, 0, speed, 0, 0, speed);
    cmdVel.publish(twist);
  }
} (window.baseModel=window.baseModel || {}));
