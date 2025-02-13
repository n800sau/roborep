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

			var self = this;

			self.config = {
				theme: "https://unpkg.com/videogular@2.1.2/dist/themes/default/videogular.css"
			};

			$scope.change_url = function(fpointer) {
				if(fpointer >= $scope.flist.length) {
					fpointer = 0;
				}
				$scope.fpointer = fpointer;
				self.config.sources = [
						{src: $sce.trustAsResourceUrl("data/input/" + $scope.flist[$scope.fpointer] + "?load_id=" + $scope.load_id), type: "video/webm"}
					];
			}

			$scope.select_file = function(fname) {
				for(var i=0; i<$scope.flist.length; i++) {
					if($scope.flist[i] == fname) {
						$scope.change_url(i);
						break;
					}
				}
			}

			$scope.fpointer = 0;
			$http.get('file.lst').then(function(response) {
				$scope.flist = response.data.trim().split(/\s/);
				$scope.change_url(0);
			});


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
