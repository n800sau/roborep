rm -rf output/clustered/*
python3 -u clustering.py &> run_clustering.log
echo $?
