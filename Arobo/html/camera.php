<?php

$r = new Redis();
$r->connect('127.0.0.1', 6379);
$cmd_json = $_GET['camera'];
$r->rpush('camera', $cmd_json);
$r->close();
echo json_encode(array('result' => 'Command "'.json_decode($cmd_json)->command.'" queued'));

?>
