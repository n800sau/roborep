<?php

	include 'vars.php';

	$start = isset($_GET["start"]) ? (int)$_GET["start"] : 0;
	$count = isset($_GET["count"]) ? (int)$_GET["count"] : 5;

	$flist = array();

	$i = 0;
	foreach(glob($SRCPATH . '/*') as $fpath) {
		$itype = exif_imagetype($fpath);
		if($itype) {
			if($i >= $start) {
				// check if thumb exists
				$bname = basename($fpath);
				$ftime = filemtime($fpath);
				$thpath = $THUMBPATH . '/' . $bname;
				$thtime = file_exists($thpath) ? filemtime($thpath) : NULL;
				if(is_null($thtime) || ($thtime < $ftime)) {
					// make thumbnail
					$thumb = new \Imagick($fpath);
					$thumb->resizeImage($TH_WIDTH, $TH_HEIGHT, imagick::INTERPOLATE_BICUBIC, -1);
					$thumb->writeImage($thpath);
				}
				$flist[] = array(
					'thumb' => $thpath,
					'image' => $fpath,
					'bname' => $bname,
				);
				$count--;
				if($count <= 0) {
					break;
				}
			}
			$i++;
		}
	}
	echo json_encode($flist);

?>
