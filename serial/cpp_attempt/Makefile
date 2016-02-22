#Makefile for serial module

FLAGS=-Wall

all: test_serial test_echo serial_raspi.o

test_serial: test_serial.o serial_raspi.o
	g++ -o test_serial test_serial.o serial_raspi.o $(FLAGS)

test_echo: test_echo.o serial_raspi.o
	g++ -o test_echo test_echo.o serial_raspi.o $(FLAGS)

test_echo.o: test_echo.cpp
	g++ -c test_echo.cpp $(FLAGS)

test_serial.o: test_serial.cpp
	g++ -c test_serial.cpp $(FLAGS)

serial_raspi.o: serial_raspi.cpp serial_raspi.h
	g++ -c serial_raspi.cpp $(FLAGS)

clean:
	rm *.o