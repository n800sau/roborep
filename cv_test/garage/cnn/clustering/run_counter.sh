rm output/npdata/*.npz
python3 -u counter.py &> run_counter.log
echo $?
