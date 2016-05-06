<?php

	include 'vars.php';

	$data = file_get_contents("php://input");
	$data = json_decode($data, TRUE);
	$fpath = $SRCPATH . '/' . $data['bname'];
	$itype = exif_imagetype($fpath);
	if($itype) {
		$npath = $NEGATIVEPATH . '/' . $data['bname'];
		$negative = file_exists($npath);
		if($data['negative'] && !$negative) {
			// create link
			symlink(realpath($fpath), $npath);
		} elseif($negative && !$data['negative']) {
			// remove link
			unlink($npath);
		}
		echo json_encode(array('negative' => file_exists($npath)));
	} else {
		header ("HTTP/1.1 404 Not Found");
		die();
	}

?>
