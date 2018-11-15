rm output/plots/*.png
python3 -u make_plots.py &> run_make_plots.log
echo $?
