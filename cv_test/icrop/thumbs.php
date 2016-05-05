<?php

	$ARATIO = 4./3;
	$SRCPATH = 'images';
	$THUMBPATH = 'thumbs';
	$NEGATIVEPATH = 'negatives';
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
				$ftime = filemtime($fpath);
				$npath = $NEGATIVEPATH . '/' . $bname;
				$negative = file_exists($npath);
				$thpath = $THUMBPATH . '/' . $bname;
				$thtime = file_exists($thpath) ? filemtime($thpath) : NULL;
				$jpath = $DRAWERSPATH . '/' . $bname . '.json';
				if(file_exists($jpath)) {
					$drawer = json_decode(file_get_contents($jpath));
					$jtime = filemtime($jpath);
				} else {
					$drawer = array();
					$jtime = NULL;
				}
				if(is_null($thtime) || ($thtime < $ftime) || ((!is_null($jtime)) && ($thtime < $jtime))) {
					// make thumbnail
					$thumb = new \Imagick($fpath);
					$draw = new \ImagickDraw();
					$strokeColor = new \ImagickPixel('red');
					$draw->setStrokeColor($strokeColor);
					$draw->setStrokeOpacity(1);
					$draw->setFillOpacity(0);
					$draw->setStrokeWidth(5);
					// draw rects
					foreach($drawer as $rect) {
						$draw->rectangle($rect->x1, $rect->y1, $rect->x2, $rect->y2);
					}
					$thumb->drawImage($draw);
					$thumb->resizeImage($TH_WIDTH, $TH_HEIGHT, imagick::INTERPOLATE_BICUBIC, -1);
					$thumb->writeImage($thpath);
				}
				$flist[] = array(
					'thumb' => $thpath,
					'image' => $fpath,
					'bname' => $bname,
					'drawer' => $drawer,
					'negative' => $negative
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
