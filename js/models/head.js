(function(headModel) {
  var actionClient;
  var pan = 0;
  var tilt = 1;

  function genGoal() {
    return new ROSLIB.Goal({
      actionClient: actionClient,
      goalMessage: {
        target: {
          header: {
            frame_id: 'base_link'
          },
          point: {
            x: 1,
            y: pan,
            z: tilt,
          }
        }
      }
    });
  }

  headModel.init = function(ros) {
    actionClient = new ROSLIB.ActionClient({
      ros: ros,
      serverName: '/head_traj_controller/point_head_action',
      actionName: 'pr2_controllers_msgs/PointHeadAction'
    });
  }

  headModel.stop = function() {
  }

  headModel.lookUp = function(speed) {
    tilt += speed;
    genGoal().send();
  }

  headModel.lookDown = function(speed) {
    tilt -= speed;
    genGoal().send();
  }

  headModel.lookLeft = function(speed) {
    pan += speed;
    genGoal().send();
  }

  headModel.lookRight = function(speed) {
    pan -= speed;
    genGoal().send();
  }
} (window.headModel=window.headModel || {}));
