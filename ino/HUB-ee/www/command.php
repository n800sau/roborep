<?php

$r = new Redis();
$r->connect('127.0.0.1', 6379);
$r->publish("command", $_GET['cmd']);
$r->close();
echo json_encode(array('result' => 'Command "'.$_GET['cmd'].'" published'));
?>
