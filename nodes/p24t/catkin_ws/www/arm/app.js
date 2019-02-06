angular.module('myApp', ['ngMaterial'])
	.controller('AppCtrl', function ($scope, $http, $timeout) {

		$scope.settings = [
				{
					s_id: 'one',
					vmin: 0,
					vmax: 100,
				},
				{
					s_id: 'two',
					vmin: 0,
					vmax: 100,
				},
				{
					s_id: 'three',
					vmin: 0,
					vmax: 100,
				},
				{
					s_id: 'four',
					vmin: 0,
					vmax: 100,
				},
			];

		var srv_map = {
			one: 'grip',
			two: 'yaw',
			three: 'upper',
			four: 'lower',
		};

		$scope.values = {
			one: 50,
			two: 50,
			three: 50,
			four: 50,
		};

		$scope.item_enabled = {
			one: false,
			two: false,
			three: false,
			four: false,
		}

		var save_values_promise = undefined;

		var save_values_debounce = function(timeout) {
			timeout = timeout || 100;
			if(save_values_promise) {
				$timeout.cancel(save_values_promise);
			}
			save_values_promise = $timeout(save_values, timeout);
		}

		var save_values = function() {
			for(var k in $scope.values) {
				var param = {
					wvalue: $scope.item_enabled[k] ? $scope.values[k] : -1,
				};
				console.log(srv_map[k], 'call with', param);
				var request = new ROSLIB.ServiceRequest(param);
				$scope[srv_map[k] + '_srv'].callService(request, function(srv_name) {
					return function(result) {
						console.log('Result for service call on ' + srv_name, result);
					};
				}(srv_map[k]));
			}
		}

		$scope.$watch('values', function(newVal, oldVal) {
			console.log('vals', oldVal, '->', newVal);
			save_values_debounce();
		}, true);

		$scope.$watch('item_enabled', function(newVal, oldVal) {
			console.log('en', oldVal, '->', newVal);
			save_values_debounce();
		}, true);

		$scope.init_ros = function() {
			// Connecting to ROS
			// -----------------
			// opcplus 34293 -> 9093
			// radxa 33293 -> 9093, 33299 -> 9094

			$scope.ros = new ROSLIB.Ros({
//				url : 'ws://p24t.local:9090'
				url : 'ws://n800s.ddns.net:33290'
			});

			$scope.ros.on('connection', function() {
				console.log('Connected to websocket server.');
			});

			$scope.ros.on('error', function(error) {
				console.log('Error connecting to websocket server: ', error);
			});

			$scope.ros.on('close', function() {
				console.log('Connection to websocket server closed.');
			});

			$scope.grip_srv = new ROSLIB.Service({
				ros : $scope.ros,
				name : 'grip',
				serviceType : 'ArmServo'
			});

			$scope.yaw_srv = new ROSLIB.Service({
				ros : $scope.ros,
				name : 'yaw',
				serviceType : 'ArmServo'
			});

			$scope.upper_srv = new ROSLIB.Service({
				ros : $scope.ros,
				name : 'upper',
				serviceType : 'ArmServo'
			});

			$scope.lower_srv = new ROSLIB.Service({
				ros : $scope.ros,
				name : 'lower',
				serviceType : 'ArmServo'
			});

		}

	});

