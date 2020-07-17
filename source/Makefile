CXX = g++
CXXFLAGS = -Wall -Werror -Wextra -pedantic -std=c++11 -g -fsanitize=address
LDFLAGS =  -fsanitize=address

SRC = program.cpp paging-simulation.h page-table.h virtual-memory.h page-repl-algorithm.h
OBJ = $(SRC:.cc=.o)
EXEC = sortArrays

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CXX) $(CXXFLAGS) program.cpp -lpthread -o sortArrays

clean:
	rm -rf $(OBJ) $(EXEC)