gcc -std=c99 -fopenmp test.c -o test && \
echo Start && \
./test &> result.log


#gcc -std=c99 test.c -o test && \
#./test &> result.log