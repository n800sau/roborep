rm -rf output/clustered/*
rm output/plots/*.png
python3 -u clustering.py &> run_clustering.log
echo $?
