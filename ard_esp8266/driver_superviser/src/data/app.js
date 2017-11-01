var app = angular.module("app", []);

app.controller("valform", ['$scope', '$http', '$interval', '$timeout', '$element', '$interval',
		function($scope, $http, $interval, $timeout, $element, $interval) {

	$http.get('current_values').then(function(response) {
		$scope.current_values = response.data;
	}).catch(function(response) {
	});

	$scope.set_param = function(name, value) {
		$http.get('set_param',{
			params: {
				name: name,
				value: value
			}
		}).then(function(response) {
			$scope.current_values[name] = response.data.value;
		}).catch(function(response) {
			$scope.error_msg = response.error;
		});
	}

	$scope.stop = function() {
		$http.get('stop',{
		}).then(function(response) {
		}).catch(function(response) {
			$scope.error_msg = response.error;
		});
	}

	$interval(function() {
		$http.get('status',{
		}).then(function(response) {
			$scope.status = response.data;
		}).catch(function(response) {
			$scope.error_msg = response.error;
		});
		$timeout(function() {
			$http.get('ypr',{
			}).then(function(response) {
				$scope.ypr = response.ypr;
			}).catch(function(response) {
				$scope.error_msg = response.error;
			});
		}, 1000);
	}, 5000);

}]);
