rm output/npdata/*_pics/*.npz
python3 -u counter.py &> run_counter.log
echo $?
