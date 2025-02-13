async function extractFramesFromVideo(videoUrl, fps=25) {
	return new Promise(async (resolve) => {

		// fully download it first (no buffering):
		let videoBlob = await fetch(videoUrl).then(r => r.blob());
		let videoObjectUrl = URL.createObjectURL(videoBlob);
		let video = document.createElement("video");

		let seekResolve;
		video.addEventListener('seeked', async function() {
			if(seekResolve) seekResolve();
		});

		video.src = videoObjectUrl;

		// workaround chromium metadata bug (https://stackoverflow.com/q/38062864/993683)
		while((video.duration === Infinity || isNaN(video.duration)) && video.readyState < 2) {
			await new Promise(r => setTimeout(r, 1000));
			video.currentTime = 10000000*Math.random();
		}
		let duration = video.duration;

		let canvas = document.getElementById('video');
		let context = canvas.getContext('2d');
		let [w, h] = [video.videoWidth, video.videoHeight]
//		canvas.width =	w;
//		canvas.height = h;

		let frames = [];
		let interval = 1 / fps;
		let currentTime = 0;

		while(currentTime < duration) {
			video.currentTime = currentTime;
			await new Promise(r => seekResolve=r);

			context.drawImage(video, 0, 0, w, h);
			let base64ImageData = canvas.toDataURL();

			frames.push(base64ImageData);
		crop_obj.replace(base64ImageData, true)
//			var img = document.getElementById('cropper');
//			img.src = base64ImageData;
			currentTime += interval;
		}
		resolve(frames);
	});

};


function rgb2hsv (r, g, b) {
	let rabs, gabs, babs, rr, gg, bb, h, s, v, diff, diffc, percentRoundFn;
	rabs = r / 255;
	gabs = g / 255;
	babs = b / 255;
	v = Math.max(rabs, gabs, babs),
	diff = v - Math.min(rabs, gabs, babs);
	diffc = c => (v - c) / 6 / diff + 1 / 2;
	percentRoundFn = num => Math.round(num * 100) / 100;
	if (diff == 0) {
		h = s = 0;
	} else {
		s = diff / v;
		rr = diffc(rabs);
		gg = diffc(gabs);
		bb = diffc(babs);

		if (rabs === v) {
			h = bb - gg;
		} else if (gabs === v) {
			h = (1 / 3) + rr - bb;
		} else if (babs === v) {
			h = (2 / 3) + gg - rr;
		}
		if (h < 0) {
			h += 1;
		}else if (h > 1) {
			h -= 1;
		}
	}
	return {
		h: Math.round(h * 360),
		s: percentRoundFn(s * 100),
		v: percentRoundFn(v * 100)
	};
}

var xvals = [];

var xmap = [];

var ch = new Chart(document.getElementById('plot').getContext('2d'), {
	type: 'bar',
	options: {
		onClick: function(ev, actives) {
			if(actives.length > 0) {
				var elementIndex = actives[0]._datasetIndex;
				var idx = actives[0]._index
//				console.log("elementIndex: " + elementIndex + "; array length: " + actives[0].length);
				console.log('click', idx);
//				var chartData = actives[elementIndex]['_chart'].config.data;
				var chartData = actives[0]['_chart'].config.data;
				var label = chartData.labels[idx];
				var value = chartData.datasets[elementIndex].data[idx];
				var series = chartData.datasets[elementIndex].label;
				console.log(series + ':' + label + ':' + value);
				var val = prompt('enter value', '');
				if(val != null) {
					for(var i=xmap.length-1; i>0; i--) {
						if(xmap[i][0] == label) {
							xmap.splice(i, 1);
							break;
						}
					}
					xmap.push([label, val]);
					xmap.sort();
				}
			}
		},
		responsive: false,
		legend: {
			position: 'top',
		},
		title: {
			display: true,
			text: 'Chart.js Bar Chart'
		},
		tooltips: {
			// Disable the on-canvas tooltip
			enabled: false,
		},
		scales: {
			yAxes: [{
				display: false,
			}],
			xAxes: [{
				ticks: {
					maxTicksLimit: 10,
				}
			}]
		},
		animation: {
			duration: 0,
		}
	}
});

function map_x(x) {
	var rs = x;
console.log(x);
	for(var i=0; i<xmap.length-1; i++) {
		if(xmap[i][0]<=x && x<xmap[i+1][0]) {
			rs = xmap[i][1] + (xmap[i+1][1] - xmap[i][1])/(xmap[i+1][0] - xmap[i][0])*x;
			break;
		}
	}
	return rs;
}

function updatePlot() {
	var data = crop_obj.getCropBoxData();
	if(data.width !== undefined) {
		var ctx = crop_obj.getCroppedCanvas().getContext('2d');
		var imgData = ctx.getImageData(0, 0, data.width, data.height);
//		console.log(event.detail.x);
//		console.log(event.detail.y);
//		console.log(event.detail.width);
//		console.log(event.detail.height);
//		console.log(event.detail.rotate);
//		console.log(event.detail.scaleX);
//		console.log(event.detail.scaleY);
			// find brightness
			var vals = [];
			var xlist=[];
			xvals = [];
			for (var y = 0; y < data.height; y++) {
				var v = 0;
				for (var x = 0; x < data.width; x++) {
					var i = (x + data.width * y) * 4;
					var r = imgData.data[i];
					var g = imgData.data[i+1];
					var b = imgData.data[i+2];
					var hsv = rgb2hsv(r, g, b);
					v += hsv.v;
				}
				xvals.push(y)
				xlist.push(map_x(y));
				vals.push(v/data.width);
			}

			console.log('vals size', vals.length);

		var barChartData = {
			labels: xlist,
			datasets: [{
				label: 'Dataset 1',
				backgroundColor: Chart.helpers.color('gray').alpha(0.5).rgbString(),
				borderColor: Chart.helpers.color('blue'),
				borderWidth: 1,
				data: vals,
			}]
		};

		ch.data = barChartData;
		ch.update();
	}
}

function init_cropper() {

	crop_obj = new Cropper(document.getElementById('cropper'), {
		responsive: false,
		autoCrop: false,
		background: false,
		crop(event) {
			updatePlot();
		},
	});

}



run_extractor = async function(ev) {
	init_cropper();
	setInterval(updatePlot, 1000);
	let frames = await extractFramesFromVideo("https://www.radiantmediaplayer.com/media/bbb-360p.mp4");
	console.log(frames);
}
