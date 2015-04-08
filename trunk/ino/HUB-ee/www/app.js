var app = angular.module("app", []);

app.controller("robostate", ['$scope', '$http', '$interval', function($scope, $http, $interval) {
  var update_state = function() {
    $http.get('/rgbframe/state.php'
    ).success(
      function(data, status, headers, config) {
        if(data.state) {
          $scope.state = data.state;
        }
        $scope.xylist = data.xy;
      }
    ).error(
      function(data, status, headers, config) {
        console.log("State data loading failed!");
      }
    );
  };

  $interval(update_state, 3000);
  update_state();
}]);

app.directive("drawing", function(){
  return {
    restrict: "A",
    link: function(scope, elements, attrs){

      scope.$watch('xylist', function(val) {
        if(val && val.length) {
          for(var j=0; j<elements.length; j++) {
            var el = elements[j];
            var ctx = el.getContext('2d');
            ctx.save();
            ctx.beginPath();
            ctx.clearRect (0, 0, el.width, el.height);
            ctx.translate(el.width / 2, el.height / 2);
            ctx.rotate(270*Math.PI/180);
            ctx.moveTo(val[0].x, val[0].y);
            for(var i=1; i<val.length; i++) {
  //            console.log(val[i]);
              // to
              ctx.lineTo(val[i].x, val[i].y);
            }
            // color
            ctx.strokeStyle = "#4bf";
            // draw it
            ctx.stroke();
            ctx.closePath();
            ctx.restore();

            if(scope.state) {
              ctx.save();
              ctx.beginPath();
              ctx.translate(el.width / 2, el.height / 2);
//              ctx.rotate(360*Math.PI/180);
              ctx.rotate(scope.state.head*Math.PI/180);
              ctx.moveTo(0, 0);
              ctx.lineTo(0, -Math.min(el.width, el.height) / 4);
              ctx.strokeStyle = "#e00";
              ctx.stroke();
              ctx.closePath();
              ctx.restore();
            }
          }
        }
      });

    }
  };
});

