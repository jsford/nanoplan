all: src/main.cpp
	g++ src/main.cpp -o nanoplan -I./include -I/usr/local/include/ -lfmt -std=c++20 -O3

benchmark: all
	hyperfine ./nanoplan --warmup 2 --min-runs 10

clean:
	rm nanoplan
