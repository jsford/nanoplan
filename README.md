<p align="center" width="100%">
    <img width="100%" src="images/logo.svg"> 
</p>

---

**nanoplan** is a header-only C++11 library for search-based robot planning. The primary design goals are correctness, ease-of-use, and efficiency (in that order). 


<p align="center" width="100%">
    <img width="60%" src="images/demo.gif"> 
</p>

## Install

```
git clone github.com/jsford/nanoplan/
cd nanoplan
mkdir build
cd build
cmake ..
sudo cmake --build . --config Release --target install --
```

## Usage
```
project("nanoplan-example")

find_package(nanoplan CONFIG REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} nanoplan::nanoplan)
```

## Acknowledgements
Thank you to Carnegie Mellon's [Search Based Planning Lab](http://sbpl.net/) and Professor Maxim Likhachev for inventing and publishing many of the algorithms implemented in nanoplan.

The nanoplan logo is based on "[Robot thinker](https://creativemarket.com/studiostoks/604281-Robot-thinker)", original art licensed from [stduiostoks](http://www.studioks.ru/).
