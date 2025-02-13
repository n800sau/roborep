(function($) {

	var stop_json = {command: 'stop'};

	var app = angular.module("app", ['rzModule', 'angularSpinner']);

	app.controller("rocontrol", ['$scope', '$http', '$interval', '$timeout', '$element', 'usSpinnerService',
			function($scope, $http, $interval, $timeout, $element, usSpinnerService) {

		$scope.brightness = 70;
		$scope.contrast = 70;
		$scope.shutter = 0;

		var param_update_timeout;

		$scope.param_update = function(cb) {
			if(param_update_timeout) {
				$timeout.cancel(param_update_timeout);
			}
			param_update_timeout = $timeout(function() {
				var cam_json = JSON.stringify({
					command: 'adjust',
					params: {
						brightness: $scope.brightness,
						contrast: $scope.contrast,
						shutter: $scope.shutter,
					}
				});
//				$scope.debug_msg = cam_json;
				usSpinnerService.spin('spinner-busy');
				$http.get('camera.php',{
					params: { camera: cam_json }
				}).success(function(data) {
					usSpinnerService.stop('spinner-busy');
					$scope.reply_message = $scope.cmd_json.command + ': ' + (data.result || data);
					if(cb) {
						cb();
					}
				}).error(function(data) {
					usSpinnerService.stop('spinner-busy');
					$scope.error_text = data;
				});
			}, 1000);
		}

		$scope.$watchGroup(['brightness', 'contrast', 'shutter'],
			function(newValues, oldValues, scope) {
				$scope.param_update();
			}
		);

		$scope.wait_till_stop = 1;
		$scope.power = 20;
		$scope.steps = 3;
		$scope.cmd_json = {};
		$scope.picam_pfx = 'picam_';
		$scope.$watch('cmd_json', function(newVal, oldVal) {
			$scope.send_command()
		});
		$scope.cmd_json = stop_json;

		$scope.send_command = function() {
			$scope.error_text = '';
			usSpinnerService.spin('spinner-busy');
			$http.get('command.php',{
				params: {
					command: $scope.cmd_json
				}
			}).success(function(data) {
				usSpinnerService.stop('spinner-busy');
				$scope.reply_message = $scope.cmd_json.command + ': ' + (data.result || data);
				if($scope.cmd_json.command != stop_json.command) {
					$timeout(
						function() {
							$scope.cmd_json = stop_json;
							$scope.primeupdate_img();
						}, $scope.wait_till_stop * 1000);
				}
			}).error(function(data) {
				usSpinnerService.stop('spinner-busy');
				$scope.error_text = data;
			});
		}

		$scope.reload_imgs = function() {
			$scope.picam_img = [];
			for(var i=0; i<20; i++) {
				$scope.picam_img.push('picam_' + i + '.jpg?time=' + (new Date()).toString());
			}
		}

		$scope.primeupdate_img = function() {
			$scope.busy = true;
			$scope.error_text = '';
			var cam_json = JSON.stringify({
				command: 'primeupdate',
			});
			usSpinnerService.spin('spinner-busy');
			$http.get('camera.php',{
				params: { camera: cam_json }
			}).success(function(data) {
				$scope.busy = false;
				usSpinnerService.stop('spinner-busy');
				$scope.reply_message = $scope.cmd_json.command + ': ' + (data.result || data);
				$timeout($scope.reload_imgs, 1000);
				try {
					if(data.result) {
						$scope.reply_message = data.result;
					} else {
						$scope.error_text = data;
					}
				} catch(err) {
					$scope.error_text = err;
				}
			}).error(function(data) {
				$scope.busy = false;
				usSpinnerService.stop('spinner-busy');
				$scope.error_text = data;
			});
		}

		$scope.update_imgs = function() {
			$scope.busy = true;
			$scope.error_text = '';
			var cam_json = JSON.stringify({
				command: 'update',
			});
			usSpinnerService.spin('spinner-busy');
			$http.get('camera.php',{
				params: { camera: cam_json }
			}).success(function(data) {
				$scope.busy = false;
				usSpinnerService.stop('spinner-busy');
				$scope.reply_message = $scope.cmd_json.command + ': ' + (data.result || data);
				$timeout($scope.reload_imgs, 1000);
				try {
					if(data.result) {
						$scope.reply_message = data.result;
					} else {
						$scope.error_text = data;
					}
				} catch(err) {
					$scope.error_text = err;
				}
			}).error(function(data) {
				$scope.busy = false;
				usSpinnerService.stop('spinner-busy');
				$scope.error_text = data;
			});
		}

		$scope.reload_imgs();

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
			$http.get('sensors.php'
			).success(
				function(data, status, headers, config) {
					if(data.result) {
						$scope.state = data.result;
					}
				}
			).error(
				function(data, status, headers, config) {
					console.log("State data loading failed!");
				}
			);
		};

		$interval(update_state, 2000);
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
