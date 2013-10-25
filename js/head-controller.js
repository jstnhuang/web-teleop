(function(headController) {
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

  headController.init = function(ros) {
    actionClient = new ROSLIB.ActionClient({
      ros: ros,
      serverName : '/head_traj_controller/point_head_action',
      actionName : 'pr2_controllers_msgs/PointHeadAction'
    });
  }

  headController.stop = function() {
  }

  headController.lookUp = function(speed) {
    tilt += speed;
    genGoal().send();
  }

  headController.lookDown = function(speed) {
    tilt -= speed;
    genGoal().send();
  }

  headController.lookLeft = function(speed) {
    pan += speed;
    genGoal().send();
  }

  headController.lookRight = function(speed) {
    pan -= speed;
    genGoal().send();
  }
} (window.headController=window.headController || {}));
