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

		var gCharts = {};


		Vue.use(VueMaterial.default);

		var v = new Vue({
			el: '#currentdate',
			data: {
				server_ts: '',
				last_data_ts: '',
				last_data_value: '',
			},
			delimiters: ['[[',']]'],
		});

/*
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
*/
		var ts_f = 'YYYY-MM-DD HH:mm:SS';

		var gTypes = ['hourly', 'daily', 'monthly'];

		var make_wells = function(search_date) {
			search_date = search_date || moment();
			var wells = {};
			var labels = {};
			gTypes.forEach(function(name) {
				wells[name] = [];
				labels[name] = [];
			});
			var h = moment(search_date).set({hour:0, minute:0, second:0, millisecond:0});
			for(var i=0; i<=24; i++) {
				wells.hourly.push(h.hour(i).format(ts_f));
				labels.hourly.push(h.format('HH'));
			}
			var m = moment(search_date).set({hour:0, minute:0, second:0, millisecond:0}).subtract(1, 'month');
			while(m.isSameOrBefore(search_date)) {
				wells.daily.push(m.add(1, 'day').format(ts_f));
				labels.daily.push(m.format('DD MMM'));
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

		String.prototype.capitalize = function() {
			return this.charAt(0).toUpperCase() + this.slice(1);
		}

		var wells = make_wells();
		var chartData = {};
		gTypes.every(function(name) {
			chartData[name] = {
				labels: [],
				datasets: [
					{
						label: 'sm',
						backgroundColor: window.chartColors.blue,
//						yAxisID: 'y-axis-smoke',
						data: [],
					}, {
						label: 'al',
						backgroundColor: window.chartColors.green,
//						yAxisID: 'y-axis-alcohol',
						data: [],
					}, {
						label: 'co',
						backgroundColor: window.chartColors.brown,
//						yAxisID: 'y-axis-co2',
						data: [],
					}
				]
			};
			chartData[name].labels = wells.labels[name];
			chartData[name].datasets[0].data = new Array(chartData[name].labels.length).fill(0);
			chartData[name].datasets[1].data = new Array(chartData[name].labels.length).fill(0);
			chartData[name].wells = wells.wells[name];
			return true;
		});


		var socket = io();
		socket.on('connect', function() {
			socket.emit('full_data_load', {data: 'I\'m connected!'});
		});
		socket.on('current_data', function(data) {
//console.log(data);
			v.server_ts = data['server_ts'];
			if(data['last']['MQ135']) {
				v.last_data_ts = data['last']['MQ135']['ts'];
				v.last_data_value = data['last']['MQ135']['value'];
			}
			gTypes.forEach(function(name) {
				data[name].forEach(function(v) {
					wells.labels[name].every(function(w, wi) {
						if(v.start_ts == wells.wells[name][wi]) {
							if(v.vals.MQ2) {
								chartData[name].datasets[0].data[wi] = v.vals.MQ2.smoke;
								chartData[name].datasets[1].data[wi] = v.vals.MQ2.alcohol;
							}
							if(v.vals.MQ135) {
								chartData[name].datasets[2].data[wi] = v.vals.MQ135.co2;
							}
							return false;
						}
						return true;
					});
				});
				gCharts[name].update();
			});
		});
		gTypes.forEach(function(name) {
			var ctx = document.getElementById(name + 'Chart').getContext('2d');
			gCharts[name] = new Chart(ctx,{
				type: 'bar',
				data: chartData[name],
				options: {
					title: {
						display: true,
						text: name.capitalize(),
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
							ticks: {
								min: 0,
								suggestedMax: 50,
							},
//							id: 'y-axis-smoke',
//						}, {
//						type: 'linear',
//						display: true,
//						position: 'right',
//						id: 'y-axis-alcohol',
//						gridLines: {
//							drawOnChartArea: false
//						}
						}],
					},
					aspectRatio: 1,
					maintainAspectRatio: false,
				}
			});
		});
	}
