Vue.use(VueMaterial.default)

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

		var ts_f = 'YYYY-MM-DD HH:mm:SS';

		var make_wells = function(search_date) {
			search_date = search_date || moment();
			var wells = {};
			var labels = {};
			v_opts.data.gTypes.forEach(function(name) {
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

		Vue.use(VueMaterial.default);
//		Vue.use(VueRouter);

		Vue.component('chart', {
			chart: undefined,
			props: ['chartOptions', 'ts'],
			template: '<canvas></canvas>',
			mounted: function() {
//				console.log('options', this.chartOptions);
				this.chart = new Chart(this.$el.getContext('2d'), this.chartOptions);
			},
			watch: {
				ts: function(newVal) {
					if(this.chart) {
						// update chart
						this.chart.update();
					}
				},
				chartOptions: function(newVal) {
					if(this.chart) {
						// update options
						this.chart.update(newVal);
					}
				}
			},
		});

		var v_opts = {
			el: '#app',
			data: {
				server_ts: '',
				card_values: {},
				card_labels: {
					temperature: '1',
					humidity: 2,
					co2: 3,
					co: 4,
				},
				card_index_list: [
					'temperature', 'humidity', 'co2', 'co',
				],
				gTypes: ['hourly', 'daily', 'monthly'],
				bar_charts: {},
				line_charts: {},
				currentTab: 'home',
			},
			delimiters: ["<%","%>"],
			created: function () {
				// `this` points to the vm instance
//				console.log('a is: ' + Object.keys(this.bar_charts))
			},
			computed: {
				currentPage() {
					if(this.$route.path == "/" || this.$route.path == "/home" ) {
						return '/home';
					} else {
						return this.$route.path;
					}
				}
			},
			methods: {
				tabChange: function(id) {
					this.currentTab = id;
				},
			}
		};

		var wells = make_wells();

		v_opts.data.gTypes.forEach(function(name) {
			v_opts.data.bar_charts[name] = {
				type: 'bar',
				data: {
					labels: wells.labels[name],
					wells: wells.wells[name],
				},
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
			};
			v_opts.data.bar_charts[name].data.datasets = [
				{
					label: 'co',
					backgroundColor: window.chartColors.blue,
					data: new Array(v_opts.data.bar_charts[name].data.labels.length).fill(0),
				}, {
					label: 'co2',
					backgroundColor: window.chartColors.green,
					data: new Array(v_opts.data.bar_charts[name].data.labels.length).fill(0),
				}
			];
		});

		v_opts.data.gTypes.forEach(function(name) {
			v_opts.data.line_charts[name] = {
				type: 'line',
				data: {
					labels: wells.labels[name],
					wells: wells.wells[name],
				},
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
			};
			v_opts.data.line_charts[name].data.datasets = [
				{
					label: 'co',
					backgroundColor: window.chartColors.blue,
					data: new Array(v_opts.data.line_charts[name].data.labels.length).fill(0),
				}, {
					label: 'co2',
					backgroundColor: window.chartColors.green,
					data: new Array(v_opts.data.line_charts[name].data.labels.length).fill(0),
				}
			];
		});

		var v = new Vue(v_opts);

		var socket = io();
		socket.on('connect', function() {
			socket.emit('full_data_load', {data: 'I\'m connected!'});
		});

		socket.on('current_data', function(data) {
//console.log('data', data);
			v.server_ts = data['server_ts'];
			v.card_values.ts = Math.min(data.last.MQ2.ts || data.last.MQ135.ts, data.last.MQ135.ts || data.last.MQ2.ts);
			if(data.last.MQ135) {
				v.card_values.co2 = data.last.MQ135.data.co2;
				v.card_values.temperature = data.last.MQ135.data.temperature;
				v.card_values.humidity = data.last.MQ135.data.humidity;
			}
			if(data.last.MQ2) {
				v.card_values.co = data.last.MQ2.data.co;
			}
			v.gTypes.forEach(function(name) {
				data[name].forEach(function(d) {
					wells.labels[name].every(function(w, wi) {
						if(d.start_ts == wells.wells[name][wi]) {
							if(d.vals.MQ2) {
								v.bar_charts[name].data.datasets[0].data[wi] = d.vals.MQ2.co;
							}
							if(d.vals.MQ135) {
								v.bar_charts[name].data.datasets[1].data[wi] = d.vals.MQ135.co2;
							}
							return false;
						}
						return true;
					});
				});
			});
		});

	}
