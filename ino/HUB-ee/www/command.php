<?php

$r = new Redis();
$r->connect('127.0.0.1', 6379);
$cmd['name'] = $_GET['cmd'];
if(!empty($_GET['params'])) {
	$cmd['params'] = json_decode($_GET['params']);
}
$r->publish("command", json_encode($cmd));
$r->close();
echo json_encode(array('result' => 'Command "'.$_GET['cmd'].'" published'));

?>
