<?php

require __DIR__.'/Predis.php';

$hname = 'dht11_data';
$r = new Predis\Client();
$time = $_POST['TIME'];
$v = (float)$_POST['V'];
$t = (float)$_POST['T'];
$h = (float)$_POST['H'];
$r->hset($hname, $time, json_encode(array('t' => $t, 'h' => $h, 'v' => $v)));
$keys = $r->hkeys($hname);
if(count($keys) > 100) {
	sort($keys, SORT_NUMERIC);
	$dkeys = array_slice($keys, 0, count($keys) - 100);
	foreach($dkeys as $dk) {
		$r->hdel($hname, $dk);
	}
}
echo 'Ok';

?>

