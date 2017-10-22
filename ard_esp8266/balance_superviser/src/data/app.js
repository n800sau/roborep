var app = angular.module("app", []);

app.controller("valform", ['$scope', '$http', '$interval', '$timeout', '$element',
		function($scope, $http, $interval, $timeout, $element) {

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
		});
	}

}]);
