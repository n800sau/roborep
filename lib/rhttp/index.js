function insertAndExecute(id, text)
{
	domelement = document.getElementById(id);
	domelement.innerHTML = text;
	var scripts = [];

	ret = domelement.childNodes;
	for ( var i = 0; ret[i]; i++ ) {
		if ( scripts && nodeName( ret[i], "script" ) && (!ret[i].type || ret[i].type.toLowerCase() === "text/javascript") ) {
			scripts.push( ret[i].parentNode ? ret[i].parentNode.removeChild( ret[i] ) : ret[i] );
		}
	}

	for(script in scripts)
	{
		evalScript(scripts[script]);
	}
}

function nodeName( elem, name ) {
	return elem.nodeName && elem.nodeName.toUpperCase() === name.toUpperCase();
}

function evalScript( elem ) {
	data = ( elem.text || elem.textContent || elem.innerHTML || "" );

	var head = document.getElementsByTagName("head")[0] || document.documentElement,
	script = document.createElement("script");
	script.type = "text/javascript";
	script.appendChild( document.createTextNode( data ) );
	head.insertBefore( script, head.firstChild );
	head.removeChild( script );

	if ( elem.parentNode ) {
		elem.parentNode.removeChild( elem );
	}
}

function json2list(placement, jsobj) {
	for(var pname in jsobj) {
		var ul_e = document.createElement('ul');
		placement.appendChild(ul_e);
		var li_e = document.createElement('li');
		ul_e.appendChild(li_e);
		var t_e = document.createElement('table');
		li_e.appendChild(t_e);
		var tr_e = document.createElement('tr');
		t_e.appendChild(tr_e);
		var td_e = document.createElement('td');
		tr_e.appendChild(td_e);
		td_e.appendChild(document.createTextNode(pname));
		var td_e = document.createElement('td');
		tr_e.appendChild(td_e);
		if(typeof(jsobj[pname]) == "object") {
			json2list(td_e, jsobj[pname]);
		} else {
			td_e.appendChild(document.createTextNode(jsobj[pname]));
		}
	}
}

var bustcachevar = 1;
var bustcacheparameter = "";
function ajaxpage(url, containerid) {
	var page_request = false
	// if Mozilla, Safari etc
	page_request = new XMLHttpRequest()
	page_request.onreadystatechange = function() {
		if (page_request.responseText) {

			var rdata = JSON.parse(page_request.responseText);
			var params_e = document.getElementById("params");
			params_e.innerHTML = '';
			json2list(params_e, rdata);
			update_image(rdata);

		}
//		try {
//			loadpage(page_request, containerid)
//		} catch (err) {
//			loadpage(err.message, containerid)
//		}
		if(nextTime) {
			clearTimeout(nextTime)
			nextTime = null;
		}
		nextTime = setTimeout(reload_info , 5000)
	}
	if (bustcachevar) {
		//if bust caching of external page
		bustcacheparameter = (url.indexOf("?") != -1) ? "&" + new Date().getTime() : "?" + new Date().getTime()
	}
	page_request.open('GET', url + bustcacheparameter, true)
	page_request.send(null)
}
function loadpage(page_request, containerid) {
	if (page_request.readyState == 4 && page_request.status == 200) {
		insertAndExecute(containerid, page_request.responseText);
	}
}
var reload_info = function() {
	var d = new Date();
	domelement = document.getElementById('curtime');
	domelement.innerHTML = d.toTimeString();
	
	//if ('WebSocket' in window){
		/* WebSocket is supported. You can proceed with your code*/
	//	document.getElementById('header').innerHtml = "websocket";
	//} else {
	//	/*WebSockets are not supported. Try a fallback method like long-polling etc*/
	//}
	nextTime = null;
	ajaxpage("/values", 'cam_values');
	nextTime = setTimeout(reload_info , 5000);
};
var img_side = new Image();
img_side.src = 'car_side';
var img_back = new Image();
img_back.src = 'car_back';
var img_top = new Image();
img_top.src = 'car_top';
var gauge = function(context, diameter, angle, imgObj)
{
	var radius = diameter/2;
	context.save();
	context.translate(radius, radius);
	context.beginPath();
	context.lineWidth="2";
	context.arc(0, 0, radius, 0 , 2 * Math.PI, false);
	context.fillStyle = 'green';
	context.fill();
	context.lineWidth="1";
	context.strokeStyle = "black";

	if (context.setLineDash) {
		context.setLineDash([5,10]);
	}

	for(var i=0; i < 360; i+=45) {
		context.save();
		context.rotate(i * Math.PI / 180);
		context.moveTo(0, radius / 4);
		context.lineTo(0, radius * 3 / 4);
		context.restore();
	}
	context.moveTo(-radius, 0);
	context.lineTo(radius, 0);
	context.stroke();

	context.rotate(angle * Math.PI / 180);
	context.beginPath();
	context.strokeStyle = "red";
	context.lineWidth="5";
	context.moveTo(0, radius);
	context.lineTo(0, 0);
	context.stroke();
	context.drawImage(imgObj, -imgObj.width/2, -imgObj.height/2);
	context.restore();
}

var update_image = function(data) {
//	var ctx = document.getElementById('m_canvas').getContext('2d');
//	ctx.fillStyle = "red";
//	ctx.beginPath();
//	ctx.translate(70, 90);
//	ctx.rotate(data['cur_heading_degrees'] * Math.PI / 180);
//	ctx.moveTo(40, 60);
//	ctx.lineTo(0, -60);
//	ctx.lineTo(-40, 60);
//	ctx.bezierCurveTo(-40, 60, 0, 0, 40, 60); // <- this is right formula for the image on the right ->
//	ctx.moveTo(0, 0);
//	ctx.fill();
	var ctx = document.getElementById('xz_canvas').getContext('2d');
	gauge(ctx, 100, data['adxl345.js.obj']['xz_degrees'], img_side);
	var ctx = document.getElementById('yz_canvas').getContext('2d');
	gauge(ctx, 100, data['adxl345.js.obj']['yz_degrees'], img_back);
	var ctx = document.getElementById('mpu_xz_canvas').getContext('2d');
	gauge(ctx, 100, data['mpu6050.js.obj']['xz_degrees'], img_side);
	var ctx = document.getElementById('mpu_yz_canvas').getContext('2d');
	gauge(ctx, 100, data['mpu6050.js.obj']['yz_degrees'], img_back);
	var ctx = document.getElementById('kx_canvas').getContext('2d');
	gauge(ctx, 100, data['kalman.js.obj']['pitch'], img_top);
	var ctx = document.getElementById('m_canvas').getContext('2d');
	gauge(ctx, 100, data['hmc5883l.js.obj']['heading_degrees'], img_top);
	var l = document.getElementById('m_canvas_label');
	while (l.firstChild) {
		l.removeChild(l.firstChild);
	}
	var dt = new Date(data['hmc5883l.js.obj']['timestamp'] * 1000);
	l.appendChild(document.createTextNode('hmc5883l'));
	l.appendChild(document.createElement('BR'));
	l.appendChild(document.createTextNode(dt.toLocaleTimeString()));
	l.appendChild(document.createElement('BR'));
	l.appendChild(document.createTextNode(data['hmc5883l.js.obj']['heading_degrees']));
	var ctx = document.getElementById('m3110_canvas').getContext('2d');
	gauge(ctx, 100, data['mag3110.js.obj']['heading_degrees'], img_top);
};
