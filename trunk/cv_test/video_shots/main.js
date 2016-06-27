'use strict';
angular.module('myApp',
		[
			"ngSanitize",
			"com.2fdevs.videogular",
			"com.2fdevs.videogular.plugins.controls",
			"com.2fdevs.videogular.plugins.overlayplay"
		]
	)
	.controller('HomeCtrl',
		["$sce", "$http", "$scope", function ($sce, $http, $scope) {
			$scope.load_id = (new Date()).getTime();

			this.config = {
				sources: [
					{src: $sce.trustAsResourceUrl("data/input/v.webm?load_id=" + $scope.load_id), type: "video/webm"}
				],
				theme: "node_modules/videogular-themes-default/videogular.css"
			};
		}]
	).directive("buttonShoot", ["$http", "$timeout",
		function($http, $timeout) {
			return {
				restrict: "A",
				require: ["^videogular", "ngModel"],
				link: function($scope, elem, attrs, controllers) {
					var API = controllers[0];
					var modelCtrl = controllers[1];
					elem.on('click', function() {
						$http.get("shoot.php?time=" + API.currentTime).success(function(data) {
							var srclist = modelCtrl.$modelValue || [];
							var dt = new Date(data.time * 1000);
							srclist.push({
								url: data.url,
								label: dt.getMinutes() + ':' + dt.getSeconds() + '.' + dt.getMilliseconds()
							});
							modelCtrl.$setViewValue(srclist.splice(-3, 3));
//							console.log(data);
						}).error(function(data) {
							console.log(data);
						});
					});
				}
			}
		}]
	).directive("buttonStepBack", [
		function() {
			return {
				restrict: "A",
				require: "^videogular",
				link: function($scope, elem, attrs, API) {
					elem.on('click', function() {
						var value = API.currentTime - 100;
						API.stop();
						API.seekTime(value / 1000.);
//						console.log('step back to ' + value + ' , ' + API.currentTime);
					});
				}
			}
		}]
	).directive("buttonStepFwd", [
		function() {
			return {
				restrict: "A",
				require: "^videogular",
				link: function($scope, elem, attrs, API) {
					elem.on('click', function() {
						var value = API.currentTime + 100;
						API.stop();
						API.seekTime(value / 1000.);
//						console.log('step fwd to ' + value + ' , ' + API.currentTime);
					});
				}
			}
		}]
	);
