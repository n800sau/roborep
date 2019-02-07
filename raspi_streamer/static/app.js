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

		$scope.values = {
			one: 50,
			two: 50,
			three: 50,
			four: 50,
		};

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
					console.log('success', response.data);
					for(var k in $scope.values) {
						if(response.data.settings[k] !== undefined) {
							$scope.values[k] = response.data.settings[k];
						}
					}
				}, function(response) {
					console.log('error', response.data);
				}
			);
		}

		$scope.$watch('values', function(newVal, oldVal) {
			console.log('vals', oldVal, '->', newVal);
			save_values_debounce();
		}, true);

		load_values();

	});

