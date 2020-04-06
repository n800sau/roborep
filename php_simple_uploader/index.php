<!DOCTYPE html>
<html>
<head>
	<title>Upload your files</title>
</head>
<body>
	<div>
		<button onclick="window.location.href='.'">Reload</button>
	</div>
<?php
	if(empty($_FILES['uploaded_file'])) {
?>
	<form enctype="multipart/form-data" action="." method="POST">
		<p>Upload a file</p>
		<input type="file" name="uploaded_file"></input><br />
		<input type="submit" value="Upload"></input>
	</form>
<?php
	} else {
		$path = "uploads/";
		$path = $path . basename( $_FILES['uploaded_file']['name']);

		if(move_uploaded_file($_FILES['uploaded_file']['tmp_name'], $path)) {
			echo "The file ".       basename( $_FILES['uploaded_file']['name']). 
			" has been uploaded";
		} else{
			echo "There was an error uploading the file, please try again!";
		}
	}
?>
</body>
</html>
