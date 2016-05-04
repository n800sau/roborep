<?php

	$ARATIO = 4./3;
	$SRCPATH = 'images';
	$THUMBPATH = 'thumbs';
	$DRAWERSPATH = 'drawers';
	$TH_WIDTH = 64;
	$TH_HEIGHT = $TH_WIDTH  / $ARATIO;

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
				$thpath = $THUMBPATH . '/' . $bname;
				$thtime = filemtime($thpath);
				if((!$thtime) || ($thtime < filemtime($fpath))) {
					// make thumbnail
					$thumb = new \Imagick($fpath);
					$thumb->resizeImage($TH_WIDTH, $TH_HEIGHT, imagick::INTERPOLATE_BICUBIC, -1);
					$thumb->writeImage($thpath);
				}
				$jpath = $DRAWERSPATH . '/' . $bname . '.json';
				$flist[] = array(
					'thumb' => $thpath,
					'image' => $fpath,
					'bname' => $bname,
					'drawer' => file_exists($jpath) ? json_decode(file_get_contents($jpath)) : array()
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
