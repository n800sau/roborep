<?php

	include 'vars.php';

	if(isset($_GET["time"])) {
		$time = ((int)$_GET["time"]) / 1000.;
		$rs = array('time' => $time);
		$bname = basename($HQ_VIDEO . '_' . $time . '.jpg');
		if(!file_exists($O_FULL)) {
			mkdir($O_FULL, 0777, true);
		}
		if(!file_exists($O_SMALL)) {
			mkdir($O_SMALL, 0777, true);
		}
		$full_fname = $O_FULL . '/' . $bname;
		$small_fname = $O_SMALL . '/' . $bname;
		$rs['cmd'] = 'ffmpeg -ss ' . $time . ' -i ' . $HQ_VIDEO . ' -vframes 1 ' . $full_fname;
		$output = array();
		exec($rs['cmd'], $output);
		sleep(1);
		if(file_exists($full_fname)) {
			$small = new \Imagick($full_fname);
			$small->resizeImage($SM_WIDTH, $SM_HEIGHT, imagick::INTERPOLATE_BICUBIC, -1);
			$small->writeImage($small_fname);
			if(file_exists($small_fname)) {
				$rs['url'] = $small_fname;
			}
		} else {
			$rs['error'] = $output;
		}
	} else {
		$rs = array('error' => 'No time provided');
	}

	echo json_encode($rs);

?>
