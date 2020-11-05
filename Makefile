CC=g++
CFLAGS=-O3 -std=c++14
LDFLAGS=-O3 -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio #-lpagmo
INCLUDES=-I./include/ #-I/Users/av/local_packages/pagmo2_installed/include #-I/usr/local/include/eigen3
SRC_EXAMPLE=$(wildcard src/example/*.cpp) 
OBJ_EXAMPLE=$(SRC_EXAMPLE:.cpp=.o)
SRC_SOCIAL_DIST=$(wildcard src/social_dist/*.cpp) 
OBJ_SOCIAL_DIST=$(SRC_SOCIAL_DIST:.cpp=.o)

all: cwaggle_example 
#all: cwaggle_social_dist

cwaggle_example:$(OBJ_EXAMPLE) Makefile
	$(CC) $(OBJ_EXAMPLE) -o ./bin/$@ $(LDFLAGS)

cwaggle_social_dist:$(OBJ_SOCIAL_DIST) Makefile
	$(CC) $(OBJ_SOCIAL_DIST) -o ./bin/$@ $(LDFLAGS)

.cpp.o:
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@

clean:
	rm $(OBJ_EXAMPLE) $(OBJ_SOCIAL_DIST) bin/cwaggle_example bin/cwaggle_social_dist
