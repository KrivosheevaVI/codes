all: graph_coloring

graph_coloring: main.o
	g++ -o graph_coloring main.o

main.o: main.cpp
	g++ -c main.cpp

clean:
	rm -rf main.o graph_coloring
