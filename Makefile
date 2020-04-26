
all:
	g++ maze.cpp solve.cpp -std=c++1z -o maze

debug:
	g++ maze.cpp solve.cpp -std=c++1z -o maze -g

mac:
	clang++ -std=c++17 maze.cpp solve.cpp -o maze

mac-debug:
	clang++ -std=c++17 maze.cpp solve.cpp -o maze -g
