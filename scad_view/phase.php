<?php

	require __DIR__.'/Predis.php';
	include 'vars.php';


	$angle = isset($_GET['angle']) ? intval($_GET['angle']) : 0;
	$z = isset($_GET['z']) ? intval($_GET['z']) : 200;
	$distance = isset($_GET['distance']) ? intval($_GET['distance']) : 200;

	$r = new Predis\Client();
	$r->set('phase.angle', $angle);
	$r->set('phase.distance', $distance);
	$r->set('phase.z', $z);

	$a = $angle / 180 * pi();
	$x = intval(sin($a) * $distance);
	$y = intval(cos($a) * $distance);
	$cmd = 'openscad ' .
			'--imgsize=160,120 ' .
			'--autocenter --preview ' .
			"--camera=$x,$y,$z,0,0,0 " .
			'-o "' . $DSTIMAGE . '" "' . $SCADFILE . '"';
	putenv('DISPLAY=:5');
	unlink($DSTIMAGE);
	if(exec($cmd, $output) != 0) {
		error_log(implode(',', $output));
	}
	header('Content-Type: application/json');
	echo json_encode(array('load_id' => strftime('%H:%M:%S', intval($_GET['load_id'])), 'angle' => $angle, 'x' => $x, 'y' => $y));

?>
