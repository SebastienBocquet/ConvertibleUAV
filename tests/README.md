test1:
g++ -o test1.exe main_test1.cpp ../libDCM/mathlibNAV.c -L /usr/lib -I/usr/include -I../../googletest/googletest/include -I../libSTM -I../libUDB -lgtest -lpthread

test2:
g++ -o a.exe main_test1.cpp ../MatrixPilot/motorCntrl.c -L /usr/lib -I/usr/include -I../../googletest/googletest/include -I../libSTM -I../libUDB -lgtest -lpthread
