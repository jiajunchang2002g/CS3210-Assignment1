CXX := g++
CXXFLAGS := -Wall -Wextra -Werror -pedantic -std=c++20 -fopenmp 
RELEASEFLAGS := -Ofast

SRCS := simulation.cc common.cc traffic.cc
HEADERS := common.h traffic.h
OBJS := $(SRCS:.cc=.o)

.PHONY: all clean

all: sim.debug sim.perf

# Debug build
sim.debug: $(SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) -DDEBUG -g $(SRCS) -o $@

# Perf (release) build
sim.perf: $(SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(RELEASEFLAGS) $(SRCS) -o $@

clean:
	rm -f sim.debug sim.perf $(OBJS)

