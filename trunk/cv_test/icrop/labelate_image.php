<?php

	include 'vars.php';

	$data = file_get_contents("php://input");
	$data = json_decode($data, TRUE);
	$fpath = $SRCPATH . '/' . $data['bname'];
	$itype = exif_imagetype($fpath);
	if($itype) {
		$labels = array();
		foreach($ALL_LABELS as $label) {
			$lpath = $LABELSPATH . '/' . $label . '/' . $data['bname'];
			$val = file_exists($lpath);
			if(!isset($data['labels'][$label])) {
				$data['labels'][$label] = FALSE;
			}
			if($data['labels'][$label] && !$val) {
				// create link
				symlink(realpath($fpath), $lpath);
			} elseif($val && !$data['labels'][$label]) {
				// remove link
				unlink($lpath);
			}
			$labels[$label] = file_exists($lpath);
		}
		echo json_encode(array('labels' => $labels));
	} else {
		header ("HTTP/1.1 404 Not Found");
		die();
	}

?>
