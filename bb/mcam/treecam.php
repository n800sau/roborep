<?php

/* Version : 1.0.3
 * Date : 2013-08-24
 * File : /var/www/webcam.php
 * Info : http://www.dronkert.net/rpi/webcam.html
 */

$now = time();     // Current time on the server
clearstatcache();  // To get fresh results for all file-info functions
$basedir = '/home/n800s/public_html/mcam';
$cam = $basedir . '/output/webcam.jpg';  // Where raspistill will save every image
$buf = $basedir . '/output/buffer.jpg';  // Buffer to avoid read/write collisions, also added text
$err = $basedir . '/output/error.jpg';   // Error message if reading $cam or $buf failed


// Get file modification time of the camera image
$t_cam = 0;
if (file_exists($cam))
	if (is_readable($cam))
		$t_cam = filemtime($cam);

// Get file modification time of the image buffer
$t_buf = 0;
if (file_exists($buf))
	if (is_readable($buf))
		$t_buf = filemtime($buf);

// If camera image not found or not accessible
if (!$t_cam)
	$t_cam = $now;  // For the file name time-stamp

// JPEG image headers, time-stamped file name
header('Content-Type: image/jpeg');
header('Content-Disposition: inline; filename="webcam_' . date('Ymd_His', $t_cam) . '.jpg"');

// If the camera image is newer than the buffer and can be read
if ($t_cam > $t_buf && $im = @imagecreatefromjpeg($cam)) {

	// Save a new buffer
	$black = @imagecolorallocate($im, 0, 0, 0);
	$white = @imagecolorallocate($im, 255, 255, 255);
	$font = 1;  // Small font
	$text = date('r', $t_cam);  // ISO format date/time
	@imagestring($im, $font, 2, 1, $text, $black);  // Bottom-right shadow
	@imagestring($im, $font, 1, 0, $text, $white);
	@imagejpeg($im, $buf);  // Save to disk
	@imagejpeg($im);  // Output to browser (avoid another disk read)
	@imagedestroy($im);
	exit();  // End program early

}


// We are here if camera image not newer than buffer (or both not found)
// or reading the camera image failed

// If buffer not found or not accessible
// or reading+outputting the old buffer fails
// or reading+outputting existing error image fails
if (!$t_buf || !@readfile($buf) || !@readfile($err)) {

	// Create an error image
	$width  = 800;  // Should ideally be the same size as raspistill output
	$height = 600;

	if ($im = @imagecreatetruecolor($width, $height)) {

		$white = @imagecolorallocate($im, 255, 255, 255);
		$black = @imagecolorallocate($im, 0, 0, 0);
		@imagefilledrectangle($im, 0, 0, $width - 1, $height - 1, $white);  // Background
		@imagerectangle($im, 0, 0, $width - 1, $height - 1, $black);  // Border
		$font = 3;  // Larger font
		$text = 'Read Error';  // Message
		$tw = @imagefontwidth($font) * strlen($text);  // Text width
		$th = @imagefontheight($font);
		$x = (int)round(($width - $tw) / 2);  // Top-left text position for centred text
		$y = (int)round(($height - $th) / 2);
		@imagestring($im, $font, $x, $y, $text, $black);  // Write message in image
		@imagejpeg($im, $err);  // Save to disk
		@imagejpeg($im);  // Output to browser
		@imagedestroy($im);

	}
}

?>