angular.module('myApp', ['ngMaterial'])
	.controller('AppCtrl', function ($scope, $http, $timeout) {

		$scope.settings = [
				{
					s_id: 'one',
					vmin: 0,
					vmax: 180,
				},
				{
					s_id: 'two',
					vmin: 0,
					vmax: 180,
				},
				{
					s_id: 'three',
					vmin: 0,
					vmax: 180,
				},
				{
					s_id: 'four',
					vmin: 0,
					vmax: 180,
				},
			];

		var srv_map = {
			one: 'grip',
			two: 'yaw',
			three: 'upper',
			four: 'lower',
		};

		$scope.values = {
			one: 90,
			two: 90,
			three: 90,
			four: 90,
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
				var request = new ROSLIB.ServiceRequest(param);
				$scope[srv_map[k] + '_srv'].callService(request, function(result) {
					console.log('Result for service call on ' + srv_map[k], result);
				});
			}
		}

		var save_settings_promise = undefined;

		var save_settings_debounce = function(timeout) {
			timeout = timeout || 1000;
			if(save_settings_promise) {
				$timeout.cancel(save_settings_promise);
			}
			save_settings_promise = $timeout(save_settings, timeout);
		}

		var save_settings = function() {
			$http.post('store_settings', {settings: $scope.settings}).then(function(response) {
				console.log('success', response.data);
			}, function(response) {
				console.log('error', response.data);
			});
		}

		$scope.$watch('values', function(newVal, oldVal) {
			console.log('vals', oldVal, '->', newVal);
			save_values_debounce();
		}, true);

		$scope.$watch('item_enabled', function(newVal, oldVal) {
			console.log('en', oldVal, '->', newVal);
			save_values_debounce();
		}, true);

		$scope.$watch('settings', function(newVal, oldVal) {
			console.log('changed from', oldVal, ' to ', newVal);
			save_settings_debounce();
		}, true);

		var load_settings = function() {
			$http.get('settings').then(function(response) {
				$scope.settings = response.data.settings;
				console.log('load success', response.data);
			}, function(response) {
				console.log('load error', response.data);
			});
		}

		var init_ros = function() {
			// Connecting to ROS
			// -----------------
			// opcplus 34293 -> 9093
			// radxa 33293 -> 9093, 33299 -> 9094

			$scope.ros = new ROSLIB.Ros({
				url : 'ws://p24t.local:9090'
//				url : 'ws://n800s.ddns.net:33293'
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
				ros : ros,
				name : 'grip',
				serviceType : 'ArmServo'
			});

			$scope.yaw_srv = new ROSLIB.Service({
				ros : ros,
				name : 'yaw',
				serviceType : 'ArmServo'
			});

			$scope.upper_srv = new ROSLIB.Service({
				ros : ros,
				name : 'upper',
				serviceType : 'ArmServo'
			});

			$scope.lower_srv = new ROSLIB.Service({
				ros : ros,
				name : 'lower',
				serviceType : 'ArmServo'
			});

		}

		load_settings();
		init_ros();

	});

