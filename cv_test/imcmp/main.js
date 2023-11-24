var myApp = angular.module('myApp', []);
myApp.controller('MyController', function($scope, $http, $interval) {

	$scope.start = 0;
	$scope.count = 6;

	$scope.el = null;

	$scope.load_id = (new Date()).getTime();

	var reloadData = function() {
		$http.get('thumbs.php?load_id=' + $scope.load_id + '&start=' + $scope.start + '&count=' + $scope.count).success(
			function(data) {
				$scope.data = data.data;
				$scope.first = data.first;
				$scope.last = data.last;
			}
		);
	}

	reloadData();

	$scope.thumb_click = function(el) {
		$scope.el = el;
	}

	$scope.nextPage = function() {
		$scope.start += $scope.count;
		$scope.el = null;
		reloadData();
	}

	$scope.prevPage = function() {
		$scope.start -= $scope.count;
		if($scope.start < 0) {
			$scope.start = 0;
		}
		$scope.el = null;
		reloadData();
	}

});
