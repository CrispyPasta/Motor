CFLAGS = -pthread -lJetsonGPIO

make: main.cpp PID.o
	g++ -static main.cpp PID.o -o motor.o $(CFLAGS)

PID.o: ./PID/PID.cpp ./PID/PID.h
	g++ -c ./PID/PID.cpp -o PID.o $(CFLAGS)

run:
	./motor.o

clean:
	rm -f *.o