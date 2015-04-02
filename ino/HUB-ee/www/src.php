<?php

$r = new Redis();
$r->connect('127.0.0.1', 6379);

header('Content-Type: application/json');

$data = array();
foreach($r->lrange('xy', 0, -1) as $xy) {
	$data[] = json_decode($xy);
}
echo json_encode($data);

$r->close();

?>
