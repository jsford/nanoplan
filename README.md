<p align="center" width="100%">
    <img width="100%" src="images/logo.svg"> 
</p>

---

**nanoplan** is a header-only C++11 library for search-based robot planning. The primary design goals are correctness, ease-of-use, and efficiency (in that order). 

## Demo
The demo included with nanoplan will generate a 2d maze and search for a path from the top-left to the bottom-right. New obstacles appear with each iteration. Depending on the search algorithm you select, replanning will be much more efficient than planning from scratch.

<p align="center" width="100%">
    <img width="60%" src="images/demo.gif"> 
</p>

## Installation
```
git clone github.com/jsford/nanoplan/
cd nanoplan
mkdir build && cd build
cmake ..
sudo make install
```

## Usage

**CMake find_package**

Use CMake and find_package to consume an installed version of nanoplan.

```
project("nanoplan-example")

find_package(nanoplan CONFIG REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} nanoplan::nanoplan)
```

**CMake add_subdirectory**

Alternatively, you can use add_subdirectory to include nanoplan in your project without installing it to the system.
```
project("nanoplan-example")

add_subdirectory(nanoplan)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} nanoplan::nanoplan)
```

**Header-only**

You can always grab the entire [include/nanoplan/](https://github.com/jsford/nanoplan/tree/master/include/nanoplan) directory and drop it into your project.

## Acknowledgements
Thank you to Carnegie Mellon's [Search Based Planning Lab](http://sbpl.net/) and Professor Maxim Likhachev for inventing and publishing many of the algorithms implemented in nanoplan.

The nanoplan logo is based on "[Robot thinker](https://creativemarket.com/studiostoks/604281-Robot-thinker)", original art licensed from [stduiostoks](http://www.studioks.ru/).
