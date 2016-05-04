<?php

	$SRCPATH = 'images';
	$DRAWERSPATH = 'drawers';

	$data = file_get_contents("php://input");
	$data = json_decode($data, TRUE);
	$fpath = $SRCPATH . '/' . $data['bname'];
	$itype = exif_imagetype($fpath);
	if($itype) {
		$jpath = $DRAWERSPATH . '/' . $data['bname'] . '.json';
		$drawer = file_exists($jpath) ? json_decode(file_get_contents($jpath)) : array();
		$drawer[] = array(
			'x1' => $data['selector']['x1'],
			'y1' => $data['selector']['y1'],
			'x2' => $data['selector']['x2'],
			'y2' => $data['selector']['y2'],
		);
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
