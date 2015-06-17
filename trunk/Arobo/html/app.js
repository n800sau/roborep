(function($) {

	var app = angular.module("app", ['rzModule', 'angularSpinner']);

	app.controller("rocontrol", ['$scope', '$http', '$interval', '$timeout', '$element', 'usSpinnerService',
			function($scope, $http, $interval, $timeout, $element, usSpinnerService) {

		$scope.brightness = 50;
		$scope.contrast = 50;
		$scope.command = null;
		$scope.$watch('command', function(newVal, oldVal) {
			$scope.send_command()
		});
		$scope.command = 'stop';
		$scope.command_args = {};

		$scope.send_command = function() {
			$http.get('command.php',{
				params: {
					cmd: $scope.command,
					params: $scope.command_args
				}
			}).success(function(data) {
				$scope.reply_message = $scope.command + ':' + (data.result || data);
				if($scope.command != 'stop') {
					$timeout(function() { $scope.command = 'stop' }, 1000);
				}
			}).error(function(data) {
				$scope.error_text = data;
			});
		}

		$scope.reload_img = function() {
			$scope.imgsrc = "image000.jpg?" + new Date().getTime();
		}

		$scope.update_img = function(uri) {
			$scope.busy = true;
			usSpinnerService.spin('spinner-busy');
			$scope.reply_message = '';
			$scope.error_text = '';
			$http.get('/app/' + uri, {
				params: {
					brightness: $scope.brightness,
					contrast: $scope.contrast
				}
			}).success(function(data, status) {
				usSpinnerService.stop('spinner-busy');
				$scope.busy = false;
				$scope.reload_img();
				try {
					if(data.result) {
						$scope.reply_message = data.result;
					} else {
						$scope.error_text = data;
					}
				} catch(err) {
					$scope.error_text = err;
				}
			}).error(function(data, status) {
				usSpinnerService.stop('spinner-busy');
				$scope.busy = false;
				$scope.error_text = data;
			});
		}

		$scope.reload_img();

		$scope.set_direction = function(event) {
			 var el = event.currentTarget;
			 if(event.offsetX!==undefined){
				 var dirX = event.offsetX;
				 var dirY = event.offsetY;
			 } else { // Firefox compatibility
				 var dirX = event.layerX - el.offsetLeft;
				 var dirY = event.layerY - el.offsetTop;
			 }
			 dirX -= el.width / 2;
			 dirY -= el.height / 2;
			 $scope.dir_angle = Math.atan2(dirY, dirX) + Math.PI / 2;

			 updateDrawing($scope, [el]);

			 send_command('turn', {rad: $scope.dir_angle});

		}

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

//		$interval(update_state, 3000);
//		update_state();
	}]);

	app.directive("drawing", function() {
		return {
			restrict: "A",
			link: function(scope, elements, attrs){

				scope.$watch('xylist', function() {
					updateDrawing(scope, elements);
				});

			}
		};
	});

	var updateDrawing = function(scope, elements) {
		for(var j=0; j<elements.length; j++) {

			var el = elements[j];
			var ctx = el.getContext('2d');
			ctx.clearRect (0, 0, el.width, el.height);

			if(scope.xylist) {
				ctx.save();
				ctx.beginPath();
				ctx.translate(el.width / 2, el.height / 2);
				ctx.rotate(270*Math.PI/180);
				ctx.moveTo(scope.xylist[0].x, scope.xylist[0].y);
				for(var i=1; i<scope.xylist.length; i++) {
					// to
					ctx.lineTo(scope.xylist[i].x, scope.xylist[i].y);
				}
				// color
				ctx.strokeStyle = "#4bf";
				// draw it
				ctx.stroke();
				ctx.closePath();
				ctx.restore();
			}

			if(scope.state) {
				ctx.save();
				ctx.beginPath();
				ctx.translate(el.width / 2, el.height / 2);
				ctx.rotate(scope.state.head*Math.PI/180);
				drawLineArrow(ctx, 0, 0, 0, -Math.min(el.width, el.height) / 4);
				ctx.strokeStyle = "#e00";
				ctx.stroke();
				ctx.closePath();
				ctx.restore();
			}

			if(typeof(scope.dir_angle) == 'number') {
				ctx.save();
				ctx.beginPath();
				ctx.translate(el.width / 2, el.height / 2);
				ctx.rotate(scope.dir_angle);
				drawLineArrow(ctx, 0, 0, 0, -Math.min(el.width, el.height) / 6);
				ctx.strokeStyle = "#0e0";
				ctx.stroke();
				ctx.closePath();
				ctx.restore();
			}

		}
	}

})(jQuery);
