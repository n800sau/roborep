ls -1 IMG*.jpg|xargs -I@ -n1 convert @ -scale 30% -gravity center -background black -extent '180x100%' extended_@
