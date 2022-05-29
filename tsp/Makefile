all: salesman

salesman: main.o
	g++ -o salesman main.o

main.o: main.cpp
	g++ -c main.cpp

clean:
	rm -rf main.o salesman
