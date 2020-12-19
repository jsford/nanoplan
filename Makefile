all: src/main.cpp
	g++ src/main.cpp -o nanoplan -I./include -I/usr/local/include/ -lfmt -O3 -std=c++20

benchmark: all
	hyperfine ./nanoplan --warmup 2 --min-runs 10

clean:
	rm nanoplan
