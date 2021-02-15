CC=clang++
CFLAGS=-O3 -std=c++17
LDFLAGS=-O3 -lsfml-graphics -lsfml-window -lsfml-system #-lsfml-audio #-lpagmo
INCLUDES=-I./include/ -I./src/utils/ #-I/Users/av/local_packages/pagmo2_installed/include #-I/usr/local/include/eigen3
SRC_EXAMPLE=$(wildcard src/example/*.cpp) 
OBJ_EXAMPLE=$(SRC_EXAMPLE:.cpp=.o)
SRC_MC_ORBIT=$(wildcard src/mc_orbit/*.cpp) 
OBJ_MC_ORBIT=$(SRC_MC_ORBIT:.cpp=.o)
SRC_LASSO=$(wildcard src/lasso/*.cpp) 
OBJ_LASSO=$(SRC_LASSO:.cpp=.o)
SRC_UORBIT=$(wildcard src/uorbit/*.cpp) 
OBJ_UORBIT=$(SRC_UORBIT:.cpp=.o)

#all: cwaggle_example 
all: cwaggle_lasso

cwaggle_example:$(OBJ_EXAMPLE) Makefile
	$(CC) $(OBJ_EXAMPLE) -o ./bin/$@ $(LDFLAGS)

cwaggle_mc_orbit:$(OBJ_MC_ORBIT) Makefile
	$(CC) $(OBJ_MC_ORBIT) -o ./bin/$@ $(LDFLAGS)

cwaggle_lasso:$(OBJ_LASSO) Makefile
	$(CC) $(OBJ_LASSO) -o ./bin/$@ $(LDFLAGS)

cwaggle_uorbit:$(OBJ_UORBIT) Makefile
	$(CC) $(OBJ_UORBIT) -o ./bin/$@ $(LDFLAGS)

.cpp.o:
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@

clean:
	rm $(OBJ_EXAMPLE) $(OBJ_MC_ORBIT) $(OBJ_LASSO) $(OBJ_UORBIT) bin/cwaggle_example bin/cwaggle_mc_orbit bin/cwaggle_lasso bin/cwaggle_uorbit
