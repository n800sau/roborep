window.chartColors = {
	red: 'rgb(255, 99, 132)',
	orange: 'rgb(255, 159, 64)',
	yellow: 'rgb(255, 205, 86)',
	green: 'rgb(75, 192, 192)',
	blue: 'rgb(54, 162, 235)',
	purple: 'rgb(153, 102, 255)',
	grey: 'rgb(201, 203, 207)'
};

	initApp = function() {

		Vue.use(VueMaterial.default)

		new Vue({
			el: '#app',
			name: 'BasicDatepicker',
			data: () => ({
				selectedDate: null
			}),
			computed: {
				firstDayOfAWeek: {
					get () {
						return this.$material.locale.firstDayOfAWeek
					},
					set (val) {
						this.$material.locale.firstDayOfAWeek = val
					}
				},
				dateFormat: {
					get () {
						return this.$material.locale.dateFormat
					},
					set (val) {
						this.$material.locale.dateFormat = val
					}
				}
			}
	 	})

		var ts_f = 'DD/MM/YYYY HH:mm:SS';

		var make_wells = function(search_date) {
			search_date = search_date || moment();
			var wells = {
				hourly: [],
				daily: [],
				monthly: [],
			};
			var labels = {
				hourly: [],
				daily: [],
				monthly: [],
			};
			var h = moment(search_date).set({hour:0, minute:0, second:0, millisecond:0});
				for(var i=0; i<=24; i++) {
					wells.hourly.push(h.hour(i).format(ts_f));
				}
				var m = moment(search_date).set({hour:0, minute:0, second:0, millisecond:0}).subtract(1, 'month');
				while(m.isSameOrBefore(search_date)) {
					wells.daily.push(m.add(1, 'day').format(ts_f));
					labels.daily.push(m.format('MMM DD'));
				}
				labels.daily.splice(-1, 1);
				var y = moment(search_date).set({date:1, hour:0, minute:0, second:0, millisecond:0}).subtract(1, 'years');
				while(y.isSameOrBefore(search_date)) {
					wells.monthly.push(y.add(1, 'month').format(ts_f));
					labels.monthly.push(y.format('MMM YYYY'));
				}
				labels.monthly.splice(-1, 1);
//	console.log(wells);
				return {wells: wells, labels: labels};
			}

		var dailyChartData = {
			labels: [],
			datasets: [
				{
					label: 'sm',
					backgroundColor: window.chartColors.blue,
//					yAxisID: 'y-axis-smoke',
					data: [],
				}, {
					label: 'al',
					backgroundColor: window.chartColors.green,
//					yAxisID: 'y-axis-alcohol',
					data: [],
				}
			]
		};

		var wells = make_wells();
		dailyChartData.labels = wells.labels.daily;
		dailyChartData.datasets[0].data = new Array(dailyChartData.labels.length).fill(0);
		dailyChartData.datasets[1].data = new Array(dailyChartData.labels.length).fill(0);
		dailyChartData.wells = wells.wells.daily;

		var socket = io();
		socket.on('connect', function() {
			socket.emit('full_data_load', {data: 'I\'m connected!'});
		});
		socket.on('current_data', function(data) {
//			console.log(data);
//				dailyChartData.datasets[0].data
			data.daily.forEach(function(v) {
				wells.labels.daily.forEach(function(w, wi) {
					var ts_f = moment(v.ts_start).format('MMM DD');
					if( ts_f == w) {
						dailyChartData.datasets[0].data[wi] = v.vals.MQ2.smoke;
						dailyChartData.datasets[1].data[wi] = v.vals.MQ2.alcohol;
						return false;
					}
				});
			});

//			dailyChartData.datasets[1].data
//			myChart.data.datasets[0].data.push(data.smoke);
			myChart.update();
		});
		var ctx = document.getElementById('myChart').getContext('2d');
		var myChart = new Chart(ctx,{
			type: 'bar',
			data: dailyChartData,
			options: {
				responsive: true,
				title: {
					display: true,
					text: 'Chart.js Bar Chart - Multi Axis'
				},
				tooltips: {
					mode: 'index',
					intersect: true
				},
				scales: {
					yAxes: [{
						type: 'linear',
						display: true,
						position: 'left',
						id: 'y-axis-smoke',
//					}, {
//						type: 'linear',
//						display: true,
//						position: 'right',
//						id: 'y-axis-alcohol',
//						gridLines: {
//							drawOnChartArea: false
//						}
					}],
				},
				maintainAspectRatio: false,
			}
		});
	}
