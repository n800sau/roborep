<?php

	include 'vars.php';

	$data = file_get_contents("php://input");
	$data = json_decode($data, TRUE);
	$fpath = $SRCPATH . '/' . $data['bname'];
	$itype = exif_imagetype($fpath);
	if($itype) {
		$jpath = $DRAWERSPATH . '/' . $data['bname'] . '.json';
		$drawer = array();
		if(file_put_contents($jpath, json_encode($drawer))) {
			echo json_encode($drawer);
		} else {
			header ("HTTP/1.1 401 Unauthorized");
			die();
		}
	} else {
		header ("HTTP/1.1 404 Not Found");
		die();
	}

?>
