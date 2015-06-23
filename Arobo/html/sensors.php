<?php

$r = new Redis();
$r->connect('127.0.0.1', 6379);
$data = $r->lrange('sensors', 0, -1);
$r->ltrim('sensors', 0, -count($data));
$r->close();
echo json_encode(array('result' => ($data ? json_decode($data[0]) : NULL)));

?>
