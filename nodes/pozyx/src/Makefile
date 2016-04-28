INC=-I../include/Pozyx-Arduino-library/ -I../include/RasPiArduino/cores/piduino/ -I../include/RasPiArduino/variants/bplus/ -I../include/RasPiArduino/libraries/Wire/

test: test.cpp
	g++ -o test test.cpp -lwiringPi -lpthread

#test: test.cpp
#	g++ -MD -DRASPI $(INC) -o $@ $<

.PHONY: clean

clean:
	rm -f test
