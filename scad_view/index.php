<html>
<head>
	<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=3">
	<title><?php echo gethostname(); ?></title>
	<link rel="stylesheet" href="style.css">
	<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/angularjs/1.7.5/angular.min.js"></script>
	<script type="text/javascript">
		var myApp = angular.module('myApp', []);
		myApp.controller('MyController', function($scope, $http, $interval) {

			$scope.z = 200;
			$scope.distance = 250;
			$scope.angle = 15;

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

			$scope.move_up = function() {
				$scope.distance += 10;
				reloadData();
			}

			$scope.move_down = function() {
				$scope.distance -= 10;
				reloadData();
			}

		});
	</script>
</head>
<body>

<button onclick="location.reload()">&orarr;</button>
<br/>

<div ng-app='myApp' ng-controller='MyController' ng-cloak>

	<div>
		<table>
		<tr>
			<td>
			</td>
			<td>
				<button ng-click="move_up()" ng-disabled="processing">&up;</button>
			</td>
			<td>
			</td>
		</tr>
		<tr>
			<td>
				<button ng-click="turn_left()" ng-disabled="processing">&pr;</button>
			</td>
			<td>
			</td>
			<td>
				<button ng-click="turn_right()" ng-disabled="processing">&sc;</button>
			</td>
		</tr>
		<tr>
			<td>
			</td>
			<td>
				<button ng-click="move_down()" ng-disabled="processing">&down;</button>
			</td>
			<td>
			</td>
		</tr>
	</div>

	<div>
	{{data.load_id}}<br>{{data.angle}}<br>{{data.x}}<br>{{data.y}}
	</div>
	<img ng-src="data/images/image.png?load_id={{data.load_id}}" width="160"></img>

</div>

</body>
</html>
