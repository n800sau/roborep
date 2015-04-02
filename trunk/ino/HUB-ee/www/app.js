var app = angular.module("app", []);

app.controller("moving", function($scope, $http, $interval) {
  $interval(function() {
    $http.get('/rgbframe/src.php'
    ).success(
      function(data, status, headers, config) {
        $scope.xylist = data;
      }
    ).error(
      function(data, status, headers, config) {
        console.log("Data loading failed!");
      }
    );
  }, 1000);
});

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
//            ctx.rotate(90*Math.PI/180);
            ctx.moveTo(val[0].x,val[0].y);
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
          }
        }
      });

    }
  };
});

