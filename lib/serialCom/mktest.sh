g++ -o testSerialCom testserialcom.cpp -L. libserialCom.a -I. && \
LD_LIBRARY_PATH=. ./testSerialCom
