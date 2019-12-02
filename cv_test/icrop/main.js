var myApp = angular.module('myApp', ['mrImage']);
myApp.controller('MyController', function($scope, $http, $interval) {

	$scope.start = 0;
	$scope.count = 6;

	$scope.load_id = (new Date()).getTime();

	$scope.selector = {};

	$scope.el = null;

	var reloadData = function() {
		$http.get('thumbs.php?load_id=' + $scope.load_id + '&start=' + $scope.start + '&count=' + $scope.count).success(
			function(data) {
				$scope.data = data.data;
				$scope.labels = data.labels;
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
		if($scope.start < 0) { //>
			$scope.start = 0;
		}
		$scope.el = null;
		reloadData();
	}

	$scope.addRect = function () {
		$http.put('add_rect.php', {
			bname: $scope.el.bname,
			selector: $scope.selector
		}).success(function(data) {
			if(data) {
				$scope.el.drawer = data;
				$scope.selector.clear();
			}
		});
	};

	$scope.removeRect = function (rect) {
		$http.put('remove_rect.php', {
			bname: $scope.el.bname,
			index: $scope.el.drawer.indexOf(rect)
		}).success(function(data) {
			if(data) {
				$scope.el.drawer = data;
				$scope.selector.clear();
			}
		});
	};

	$scope.clearDrawer = function () {
		$http.put('clear_drawer.php', {
			bname: $scope.el.bname
		}).success(function(data) {
			if(data) {
				$scope.el.drawer = data;
			}
		});
	};

	$scope.negate = function(el) {
		$http.put('negate_image.php', {
			bname: el.bname,
			negative: el.negative
		}).success(function(data) {
			if(data) {
				el.negative = data.negative;
			}
		});
	}

	$scope.labelate = function(el) {
		$http.put('labelate_image.php', {
			bname: el.bname,
			labels: el.labels
		}).success(function(data) {
			if(data) {
				el.labels = data.labels;
			}
		});
	}

	$scope.is_labelled = function(el) {
		var found = false;
		for(var i=0; i<$scope.labels.length; i++) {
			if(el.labels[$scope.labels[i]]) {
				found = true;
				break;
			}
		}
		return found;
	}

});
