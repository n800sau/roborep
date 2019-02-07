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
			one: 90,
			two: 90,
			three: 90,
			four: 90,
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
				config: $scope.values
			}).then(
				function(response) {
					console.log('success', response.data);
					$scope.cam_values = response.data.current_values;
				}, function(response) {
					console.log('error', response.data);
				}
			);
		}

		var load_values = function() {
			$http.get('load_values').then(
				function(response) {
					console.log('success', response.data);
					$scope.cam_values = response.data.current_values;
				}, function(response) {
					console.log('error', response.data);
				}
			);
		}

		$scope.$watch('values', function(newVal, oldVal) {
			console.log('vals', oldVal, '->', newVal);
			save_values_debounce();
		}, true);

	});

