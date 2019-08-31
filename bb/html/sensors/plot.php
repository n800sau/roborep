<?php

require __DIR__.'/Predis.php';

$hname = 'dht11_garage';
$r = new Predis\Client();
$data = $r->hgetall($hname);
ksort($data, SORT_NUMERIC);
$data = array_reverse(array_slice($data, -100, 100, TRUE), TRUE);
$i = 0;
$odata = array();
foreach($data as $ts => $d) {
	$d = json_decode($d);
	$d->l = date('d H:i:s', (int)$ts);
	if($i % 5 == 0) {
		array_unshift($odata, $d);
	}
	$i++;
}

header('Content-type: application/json');
echo json_encode($odata, JSON_PRETTY_PRINT);

?>

