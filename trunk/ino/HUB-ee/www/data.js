init = function() {
	var xhr = new XMLHttpRequest();
	xhr.open('GET', 'data.bin', true);
	//xhr.responseType = 'blob';
	xhr.responseType = 'arraybuffer';
	xhr.onload = function(e) {
		if (this.status == 200) {
			// get binary data as a response
			//var blob = this.response;
			//console.log(blob.size);
			var data = new Uint8Array(this.response); 
			$('#canvas').each(function(idx, el) {
				var ctx = el.getContext('2d');
				ctx.clearRect (0, 0, el.width, el.height);
				var w = 60;
				var h = 80;
				var imgData=ctx.createImageData(w,h);
				var coef5 = 256/(1 << 5);
				var coef6 = 256/(1 << 6);
				for(var y=0; y<h; y++) {
					for(var x=0; x<w; x++) {
						var dp = (y*w+x)*2;
						var word = data[dp] + (data[dp+1] << 8);
//						var b1 = (word & 0x1f) * coef5;
//						var g1 = ((word >> 5) & 0x3f) * coef6;
//						var r1 = (word >> 11) * coef5;
						var b = (data[dp] & 0x1f) * coef5;
						var g = ((data[dp] >> 5) + ((data[dp+1] & 0x7) << 3)) * coef6;
						var r = (data[dp+1] >> 3) * coef5;
						dp = (y*w+x)*4;
						imgData.data[dp] = r;
						imgData.data[dp+1] = g;
						imgData.data[dp+2] = b;
						imgData.data[dp+3] = 255;
					}
				}
				ctx.putImageData(imgData,10,10);
			});
		} else {
			alert('error');
		}
	};
	xhr.send();
};
