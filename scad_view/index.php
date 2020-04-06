<html>
<head>
	<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=3">
	<title><?php echo gethostname(); ?></title>
	<link rel="stylesheet" href="style.css">
	<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/angularjs/1.7.5/angular.min.js"></script>
	<script type="text/javascript">
		var myApp = angular.module('myApp', []);
		myApp.controller('MyController', function($scope, $http, $interval, $timeout) {

<?php

	require __DIR__.'/Predis.php';
	include 'vars.php';

	$r = new Predis\Client();
	echo '$scope.angle = (' . $r->get('phase.angle') . "+ 0) || 0;\n";
	echo '$scope.distance = (' . $r->get('phase.distance') . "+ 0) || 300;\n";
	echo '$scope.z = (' . $r->get('phase.z') . "+ 0) || 200;\n";
	echo '$scope.x = (' . $r->get('phase.x') . "+ 0) || 0;\n";

?>
			var reload_deferred;

			var debounceReloadData = function() {
				if(reload_deferred) {
					$timeout.cancel(reload_deferred);
					reload_deferred = undefined;
				}
				reload_deferred = $timeout(function() {
					reloadData();
				}, 500);
			}

			var reloadData = function() {
				$scope.processing = true;
				var load_id = (new Date()).getTime();
				$http.get('phase.php?load_id=' + load_id + '&z=' + $scope.z + '&distance=' + $scope.distance + '&angle=' + $scope.angle + '&x=' + $scope.x).then(
					function(response) {
						$scope.processing = false;
						$scope.data = response.data;
//console.log('reloaded');
					}
				);
			}

			debounceReloadData();

			$scope.turn_left = function() {
				$scope.angle -= 15;
				while($scope.angle < 0) {
					$scope.angle += 360;
				}
				debounceReloadData();
			}

			$scope.turn_right = function() {
				$scope.angle += 15;
				while($scope.angle >= 360) {
					$scope.angle -= 360;
				}
				debounceReloadData();
			}

			$scope.zoom_in = function() {
				$scope.distance -= 10;
				debounceReloadData();
			}

			$scope.zoom_out = function() {
				$scope.distance += 10;
				debounceReloadData();
			}

			$scope.move_up = function() {
				$scope.z += 10;
				debounceReloadData();
			}

			$scope.move_down = function() {
				$scope.z -= 10;
				debounceReloadData();
			}

			$scope.move_left = function() {
				$scope.x -= 10;
				debounceReloadData();
			}

			$scope.move_right = function() {
				$scope.x += 10;
				debounceReloadData();
			}

		});
	</script>
</head>
<body>

<button onclick="location.reload()">&orarr;</button>

<div ng-app='myApp' ng-controller='MyController' ng-cloak>

	<div>
		<table>
		<tr>
			<td>
				<button ng-click="move_left()" ng-disabled="processing"><-</button>
			</td>
			<td>
				<button ng-click="move_up()" ng-disabled="processing">&uarr;</button>
			</td>
			<td>
				<button ng-click="move_right()" ng-disabled="processing">-></button>
			</td>
		</tr>
		<tr>
			<td>
				<button ng-click="turn_left()" ng-disabled="processing">&larr;</button>
			</td>
			<td>
			</td>
			<td>
				<button ng-click="turn_right()" ng-disabled="processing">&rarr;</button>
			</td>
		</tr>
		<tr>
			<td>
				<button ng-click="zoom_out()" ng-disabled="processing">-</button>
			</td>
			<td>
				<button ng-click="move_down()" ng-disabled="processing">&darr;</button>
			</td>
			<td>
				<button ng-click="zoom_in()" ng-disabled="processing">+</button>
			</td>
		</tr>
	</div>

	<img ng-src="data/images/image.png?load_id={{data.load_id}}" width="160"></img>

</div>

</body>
</html>
