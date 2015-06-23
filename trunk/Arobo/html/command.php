<?php

$r = new Redis();
$r->connect('127.0.0.1', 6379);
$cmd_json = $_GET['command'];
$r->publish("command", $cmd_json);
$r->close();
echo json_encode(array('result' => 'Command "'.json_decode($cmd_json)->command.'" published'));

?>
