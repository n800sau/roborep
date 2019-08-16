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
//    canvas.width =  w;
//    canvas.height = h;

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
//      var img = document.getElementById('cropper');
//      img.src = base64ImageData;
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

	var ch = new Chart(document.getElementById('plot').getContext('2d'), {
		type: 'bar',
		options: {
			onClick: function(ev, actives) {
				if(actives.length > 0) {
					console.log('click', actives[0]._index);
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

function updatePlot() {
	var data = crop_obj.getCropBoxData();
	if(data.width !== undefined) {
		var ctx = crop_obj.getCroppedCanvas().getContext('2d');
		var imgData = ctx.getImageData(0, 0, data.width, data.height);
//    console.log(event.detail.x);
//    console.log(event.detail.y);
//    console.log(event.detail.width);
//    console.log(event.detail.height);
//    console.log(event.detail.rotate);
//    console.log(event.detail.scaleX);
//    console.log(event.detail.scaleY);
			// find brightness
			var vals = [];
			var ylist=[];
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
				ylist.push(y);
				vals.push(v/data.width);
			}

			console.log('vals size', vals.length);

		var barChartData = {
			labels: ylist,
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

		do_crop = function() {
//			console.log('end', data);
//			var img = document.getElementById('cropper');
			var img = croppr.imageClippedEl;
			console.log('clipsize', img.width + 'x' + img.height);

			var ctx = document.getElementById('result_canvas').getContext('2d');
			ctx.drawImage(img, data.x, data.y, data.width, data.height, 0, 0, data.width, data.height);
			var imgData = ctx.getImageData(0, 0, data.width, data.height);
//			console.log('data', imgData);
			// find brightness
			var vals = [];
			var ylist=[];
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
				ylist.push(y);
				vals.push(v/data.width);
			}

			console.log('vals size', vals.length);

		var barChartData = {
			labels: ylist,
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

			// vals - list of mean width value
//			ctx.putImageData(imgData, data.width, 0); 
//			ctx.drawImage(img, 0, 0);
		}
	}



	run_extractor = async function(ev) {
		init_cropper();
		setInterval(updatePlot, 1000);
		let frames = await extractFramesFromVideo("https://www.radiantmediaplayer.com/media/bbb-360p.mp4");
		console.log(frames);
	}

