<html>
<head>
	<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=3">
	<title><?php echo gethostname(); ?></title>
	<link rel="stylesheet" href="style.css">
	<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/angularjs/1.4.2/angular.min.js"></script>
	<script type="text/javascript">
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
	</script>
</head>
<body>

<button onclick="location.reload()">&orarr;</button>
<br/>

<div ng-app='myApp' ng-controller='MyController' ng-cloak>

	<div>
		<button ng-click="prevPage()" ng-disabled="start <= 0">&pr;</button>
		<button ng-click="nextPage()" ng-disabled="count > data.length">&sc;</button>
	</div>

	{{start}} - {{start+count-1}}<br/>
	{{first}} - {{last}}<br/>

	<table>
		<tr class="thumb" style="width: 100px;margin: 5px; border: none;" ng-repeat="el in data">
			<td>
				<img ng-src="{{el.thumb}}" title="{{el.bname}}" ng-click="thumb_click(el)"></img>{{el.bname}}
			</td>
		</tr>
	</table>

	<img src="{{el.image}}" width="150" ng-if="el" title="{{el.image}}"></img>

</div>

</body>
</html>