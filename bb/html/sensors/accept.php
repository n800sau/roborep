<?php

require __DIR__.'/Predis.php';

$r = new Predis\Client();
$time = (int)$_POST['TIME'];
$v = (float)$_POST['V'];
$t = (float)$_POST['T'];
$h = (float)$_POST['H'];
$hname = 'dht11_' . $_POST['ID'];
$cur_time = time();
if($time == 0) {
	$time = $cur_time;
} else {
	if($cur_time + 600 < $time) {
		// timestamp too far in the future
		$warn = 'Timestamp too far in the future '.$time.' bigger than '.$cur_time.' ('.($time - $cur_time).')';
		$time = 0;
	}
}
if($time > 0) {
	$r->hset($hname, $time, json_encode(array('t' => $t, 'h' => $h, 'v' => $v)));
	$keys = $r->hkeys($hname);
	if(count($keys) > 100) {
		sort($keys, SORT_NUMERIC);
		$dkeys = array_slice($keys, 0, count($keys) - 100);
		foreach($dkeys as $dk) {
			$r->hdel($hname, $dk);
		}
	}
	echo json_encode(array('result' => 'Ok'));
} else if(isset($err)) {
	http_response_code(422);
	echo json_encode(array('error' => $err));
} else {
	http_response_code(200);
	echo json_encode(array('result' => $warn));
}

?>

