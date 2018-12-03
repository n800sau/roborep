<html>
<head>
	<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=3">
	<title><?php echo gethostname(); ?></title>
	<link rel="stylesheet" href="style.css">
	<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/angularjs/1.7.5/angular.min.js"></script>
	<script type="text/javascript">
		var myApp = angular.module('myApp', []);
		myApp.controller('MyController', function($scope, $http, $interval) {

<?php

	require __DIR__.'/Predis.php';
	include 'vars.php';

	$r = new Predis\Client();
	echo '$scope.angle = (' . $r->get('phase.angle') . "+ 0) || 0;\n";
	echo '$scope.distance = (' . $r->get('phase.distance') . "+ 0) || 300;\n";
	echo '$scope.z = (' . $r->get('phase.z') . "+ 0) || 200;\n";

?>

			var reloadData = function() {
				$scope.processing = true;
				var load_id = (new Date()).getTime();
				$http.get('phase.php?load_id=' + load_id + '&z=' + $scope.z + '&distance=' + $scope.distance + '&angle=' + $scope.angle).then(
					function(response) {
						$scope.processing = false;
						$scope.data = response.data;
//console.log('reloaded');
					}
				);
			}

			reloadData();

			$scope.turn_left = function() {
				$scope.angle -= 15;
				while($scope.angle < 0) {
					$scope.angle += 360;
				}
				reloadData();
			}

			$scope.turn_right = function() {
				$scope.angle += 15;
				while($scope.angle >= 360) {
					$scope.angle -= 360;
				}
				reloadData();
			}

			$scope.move_up = function() {
				$scope.distance -= 10;
				reloadData();
			}

			$scope.move_down = function() {
				$scope.distance += 10;
				reloadData();
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
			</td>
			<td>
				<button ng-click="move_up()" ng-disabled="processing">&uarr;</button>
			</td>
			<td>
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
			</td>
			<td>
				<button ng-click="move_down()" ng-disabled="processing">&darr;</button>
			</td>
			<td>
			</td>
		</tr>
	</div>

	<img ng-src="data/images/image.png?load_id={{data.load_id}}" width="160"></img>

</div>

</body>
</html>
