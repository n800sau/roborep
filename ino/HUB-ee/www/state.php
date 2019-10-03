<?php

$r = new Redis();
$r->connect('127.0.0.1', 6379);

header('Content-Type: application/json');

$data = array();

foreach($r->lrange('xy', 0, -1) as $xy) {
	$data['xy'][] = json_decode($xy);
}

foreach($r->lrange('state', -1, -1) as $state) {
	$data['state'] = json_decode($state);
}

echo json_encode($data);

$r->close();

?>
