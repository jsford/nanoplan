all:
	g++ src/main.cpp -o nanoplan -I./include -I/usr/local/include/ -lfmt -O3
