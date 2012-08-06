CFLAGS=-c -Wall

EIGEN3_INCLUDE_PATH=/usr/local/include/eigen3
SUITESPARSE_INCLUDE_PATH=/usr/include/suitesparse

LIBS=-lg2o_solver_csparse -lg2o_types_slam2d -lcxsparse

all: rrr

rrr: test.o 
	g++ test.o -Wall -o rrr $(LIBS)

test.o: ../test.cpp
	g++ $(CFLAGS) ../test.cpp -I$(EIGEN3_INCLUDE_PATH) -I$(SUITESPARSE_INCLUDE_PATH)
	
clean:
	rm -f *.o rrr