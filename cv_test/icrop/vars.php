<?php

	$BASEPATH = 'data';
	$ARATIO = 4./3;
	$SRCPATH = $BASEPATH . '/images';
	$THUMBPATH = $BASEPATH . '/thumbs';
	$NEGATIVEPATH = $BASEPATH . '/negatives';
	$DRAWERSPATH = $BASEPATH . '/drawers';
	$LABELSPATH = $BASEPATH . '/labels';
	$TH_WIDTH = 64;
	$TH_HEIGHT = $TH_WIDTH  / $ARATIO;

	$ALL_LABELS = array();
	foreach(glob($LABELSPATH . '/*') as $lpath) {
		$ALL_LABELS[] = basename($lpath);
	}


?>
