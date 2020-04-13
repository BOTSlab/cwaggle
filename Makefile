CC=g++
CFLAGS=-O3 -std=c++14
LDFLAGS=-O3 -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio -lpagmo
INCLUDES=-I./include/ #-I/Users/av/local_packages/pagmo2_installed/include #-I/usr/local/include/eigen3
SRC_EXAMPLE=$(wildcard src/example/*.cpp) 
OBJ_EXAMPLE=$(SRC_EXAMPLE:.cpp=.o)
SRC_ORBITAL=$(wildcard src/orbital/*.cpp) 
OBJ_ORBITAL=$(SRC_ORBITAL:.cpp=.o)
SRC_ORBITAL_AV=$(wildcard src/orbital_av/*.cpp) 
OBJ_ORBITAL_AV=$(SRC_ORBITAL_AV:.cpp=.o)
SRC_RL=$(wildcard src/rl/*.cpp) 
OBJ_RL=$(SRC_RL:.cpp=.o)

all:cwaggle_example cwaggle_orbital cwaggle_orbital_av cwaggle_rl

cwaggle_example:$(OBJ_EXAMPLE) Makefile
	$(CC) $(OBJ_EXAMPLE) -o ./bin/$@ $(LDFLAGS)

cwaggle_orbital:$(OBJ_ORBITAL) Makefile
	$(CC) $(OBJ_ORBITAL) -o ./bin/$@ $(LDFLAGS)

cwaggle_orbital_av:$(OBJ_ORBITAL_AV) Makefile
	$(CC) $(OBJ_ORBITAL_AV) -o ./bin/$@ $(LDFLAGS)
	
cwaggle_rl:$(OBJ_RL) Makefile
	$(CC) $(OBJ_RL) -o ./bin/$@ $(LDFLAGS)

.cpp.o:
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@

clean:
	rm $(OBJ_EXAMPLE) $(OBJ_ORBITAL) $(OBJ_ORBITAL_AV) $(OBJ_RL) bin/cwaggle_example bin/cwaggle_orbital bin/cwaggle_orbital_av bin/cwaggle_rl
