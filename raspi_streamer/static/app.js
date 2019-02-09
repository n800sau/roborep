angular.module('myApp', ['ngMaterial'])
	.controller('AppCtrl', function ($scope, $http, $timeout) {

		$scope.settings = [
				{
					s_id: 'brightness',
					vmin: 0,
					vmax: 100,
					stype: 'slider',
				},
				{
					s_id: 'contrast',
					vmin: 0,
					vmax: 100,
					stype: 'slider',
				},
				{
					s_id: 'iso',
					vmin: 0,
					vmax: 1600,
					step: 100,
					stype: 'slider',
				},
				{
					s_id: 'awb_gains.red',
					vmin: 0,
					vmax: 8,
					step: 0.1,
					stype: 'slider',
				},
				{
					s_id: 'awb_gains.blue',
					vmin: 0,
					vmax: 8,
					step: 0.1,
					stype: 'slider',
				},
				{
					s_id: 'awb_mode',
					values: [
						'off',
						'auto',
						'sunlight',
						'cloudy',
						'shade',
						'tungsten',
						'fluorescent',
						'incandescent',
						'flash',
						'horizon',
					],
					stype: 'select',
				},
				{
					s_id: 'drc_strength',
					values: [
						'off',
						'low',
						'medium',
						'high',
					],
					stype: 'select',
				},
				{
					s_id: 'exposure_compensation',
					vmin: -25,
					vmax: 25,
					stype: 'slider',
				},
			];

		$scope.values = {
			brightness: 50,
			contrast: 50,
			awb_mode: 'auto',
		};

		$scope.cam_values = {};

		var save_values_promise = undefined;

		var save_values_debounce = function(timeout) {
			timeout = timeout || 500;
			if(save_values_promise) {
				$timeout.cancel(save_values_promise);
			}
			save_values_promise = $timeout(save_values, timeout);
		}

		var save_values = function() {
			$http.post('apply_values', {
				settings: $scope.values
			}).then(
				function(response) {
					console.log('success', response.data);
					$scope.cam_values = response.data.settings;
				}, function(response) {
					console.log('error', response.data);
				}
			);
		}

		var load_values = function() {
			$http.get('load_values').then(
				function(response) {
					console.log('loaded', response.data.settings);
					for(var i=0;i<$scope.settings.length; i++) {
						var k = $scope.settings[i].s_id;
						if(response.data.settings[k] !== undefined) {
							$scope.values[k] = response.data.settings[k];
						}
					}
					$scope.values_loaded = true;
				}, function(response) {
					console.log('error', response.data);
					$scope.values_loaded = true;
				}
			);
		}

		$scope.$watch('values', function(newVal, oldVal) {
			console.log('vals', oldVal, '->', newVal);
			if($scope.values_loaded) {
				save_values_debounce();
			}
		}, true);

		load_values();

	});

