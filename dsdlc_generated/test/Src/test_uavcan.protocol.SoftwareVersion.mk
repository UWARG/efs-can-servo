
INC=-I../include
INC+=-ID:\Repositories\DroneCan-DSDL\libcanard
INC+=-I.
CXX=g++
CC=gcc
CFLAGS=-Wall -Wextra -Werror -std=c99 -DCANARD_DSDLC_TEST_BUILD $(INC)
CXXFLAGS=-Wall -Wextra -Werror -std=c++11 -DCANARD_DSDLC_TEST_BUILD $(INC)
LDFLAGS=-lstdc++

# get test name from Makefile name

TEST=uavcan.protocol.SoftwareVersion

TEST_SRC:=test_$(TEST).cpp
# get sources
SRCS:=$(wildcard ../src/*.c) D:\Repositories\DroneCan-DSDL\libcanard\canard.c
# generate object files
COBJS:=$(SRCS:.c=.o)
CXXOBJS+=$(TEST_SRC:.cpp=.o)

all: $(TEST)

$(TEST): $(COBJS) $(CXXOBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(COBJS) $(CXXOBJS) $(LDFLAGS)

$(COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(CXXOBJS): %.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

.PHONY: clean


clean:
	rm -f $(TEST) $(CXXOBJS) $(COBJS)
