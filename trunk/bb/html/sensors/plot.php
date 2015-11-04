<?php

require __DIR__.'/Predis.php';

$hname = 'dht11_garage';
$r = new Predis\Client();
$data = $r->hgetall($hname);
ksort($data, SORT_NUMERIC);
$data = array_slice($data, -10, 10, TRUE);
foreach($data as $ts => &$d) {
	$d = json_decode($d);
	$d->l = date('H:i:s', (int)$ts);
}

header('Content-type: application/json');
echo json_encode($data, JSON_PRETTY_PRINT);

?>

