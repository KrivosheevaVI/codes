all: knapsack

knapsack: main.o
	g++ -o knapsack main.o

main.o: main.cpp
	g++ -c main.cpp

clean:
	rm -rf main.o knapsack
