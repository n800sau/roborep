<?php

	include 'vars.php';

	function mkthumb($fpath, $thpath) {
		global $TH_WIDTH, $TH_HEIGHT;
		if(file_exists($fpath)) {
			$ftime = filemtime($fpath);
			$thtime = file_exists($thpath) ? filemtime($thpath) : NULL;
			if(is_null($thtime) || ($thtime < $ftime)) {
				if(!file_exists(dirname($thpath))) {
					mkdir(dirname($thpath), 0777, true);
				}
				// make thumbnail
				$thumb = new \Imagick($fpath);
//		error_log($TH_WIDTH . ',' . $TH_HEIGHT);
				$thumb->resizeImage($TH_WIDTH, $TH_HEIGHT, imagick::INTERPOLATE_BICUBIC, -1);
				$thumb->writeImage($thpath);
			}
		}
	}

	$start = isset($_GET["start"]) ? (int)$_GET["start"] : 0;
	$count = isset($_GET["count"]) ? (int)$_GET["count"] : 5;

	$flist = array();

	$i = 0;

	class FilesOnlyFilter extends RecursiveFilterIterator
	{
		public function accept()
		{
			$iterator = $this->getInnerIterator();

			// allow traversal
			if ($iterator->hasChildren()) {
				return true;
			}

			// filter entries, only allow true files
			return $iterator->current()->isFile();
		}
	}

	class VisibleOnlyFilter extends RecursiveFilterIterator
	{
		public function accept()
		{
			$fileName = $this->getInnerIterator()->current()->getFileName();
			$firstChar = $fileName[0];
			return $firstChar !== '.';
		}
	}

	$fileinfos = new RecursiveIteratorIterator(
		new FilesOnlyFilter(
			new VisibleOnlyFilter(
				new RecursiveDirectoryIterator($SRCPATH1, FilesystemIterator::UNIX_PATHS | FilesystemIterator::FOLLOW_SYMLINKS)
			)
		),
		RecursiveIteratorIterator::LEAVES_ONLY 
	);

	foreach ($fileinfos as $fpath => $fileinfo) {
		if(is_dir($fpath)) {
			$subdirname = dirname($fileinfos->getSubPathname());
			$bname = basename($fileinfos->getSubPathname());
			echo $fpath . ', ' . $subdirname . ', ' . $bname . "\n";
		}
	}

	foreach ($fileinfos as $dname)
	{
		$fpath1 = $dname->getPathname();
		error_log($fpath1);
		$itype = exif_imagetype($fpath1);
		if($itype) {
			if($i >= $start) {
				// check if thumb exists
				$subfname = $fileinfos->getSubPathname();
				$bname = basename($subfname);

				$thpath1 = $THUMBPATH1 . '/' . $subfname;
				mkthumb($fpath1, $thpath1);

				$fpath2 = $SRCPATH2 . '/' . $subfname;
				$thpath2 = $THUMBPATH2 . '/' . $subfname;
				mkthumb($fpath2, $thpath2);

				$fpath3 = $SRCPATH3 . '/' . $subfname;
				$thpath3 = $THUMBPATH3 . '/' . $subfname;
				mkthumb($fpath3, $thpath3);

				$bname = basename($subfname);
				$flist[] = array(
					'img1' => array(
						'thumb' => $thpath1,
						'image' => $fpath1,
						'bname' => $bname,
						'dname' => $subfname,
					),
					'img2' => array(
						'thumb' => $thpath2,
						'image' => $fpath2,
						'bname' => $bname,
						'dname' => $subfname,
					),
					'img3' => array(
						'thumb' => $thpath3,
						'image' => $fpath3,
						'bname' => $bname,
						'dname' => $subfname,
					),
				);
				$count--;
				if($count <= 0) {
					break;
				}
			}
			$i++;
		}
		if($count <= 0) {
			break;
		}
	}

	echo json_encode(array('data' => $flist, 'first' => $start, 'last' => $i));

?>
