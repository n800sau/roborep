<?php

	include 'vars.php';

	$start = isset($_GET["start"]) ? (int)$_GET["start"] : 0;
	$count = isset($_GET["count"]) ? (int)$_GET["count"] : 5;

	$flist = array();

	$i = 0;
	foreach (new RecursiveIteratorIterator(new RecursiveDirectoryIterator($SRCPATH)) as $dname)
	{
		foreach(glob($dname->getPathname() . '/*') as $fpath) {
			$it = mime_content_type($fpath);
			if($it == 'image/jpeg') {
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
						$fpathlist = explode('/', $fpath);
						$flist[] = array(
							'thumb' => $thpath,
								'image' => $fpath,
							'bname' => $bname,
							'dname' => $fpathlist[count($fpathlist)-3],
						);
						$count--;
						if($count <= 0) {
								break;
						}
					}
					$i++;
				}
			}
		}
		if($count <= 0) {
			break;
		}
	}

	echo json_encode(array('data' => $flist, 'first' => $start, 'last' => $i));

?>
