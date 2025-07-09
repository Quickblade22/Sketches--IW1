USE_SDL := 1

# This will likely need to be changed to suit your installation.
ALE := /root/Arcade-Learning-Environment/

FLAGS := -std=c++17 -I$(ALE)/src -I$(ALE)/src/controllers -I$(ALE)/src/os_dependent -I$(ALE)/src/environment -I$(ALE)/src/external -g -O0
CXX := g++
FILE := rom_planner
LDFLAGS := -L$(ALE) -lale -lz

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
    FLAGS += -Wl,-rpath=$(ALE)
endif
ifeq ($(UNAME_S),Darwin)
    FLAGS += -framework Cocoa
endif

ifeq ($(strip $(USE_SDL)), 1)
  DEFINES += -D__USE_SDL -DSOUND_SUPPORT
  FLAGS += $(shell sdl2-config --cflags)
  LDFLAGS += $(shell sdl2-config --libs)
endif

LDFLAGS += -lboost_filesystem -lboost_system -lboost_program_options

all: $(FILE)

$(FILE):	main.cc node.h planner.h sim_planner.h bfsIW.h rolloutIW.h IW.h screen.h utils.h logger.o
		$(CXX) $(DEFINES) $(FLAGS) main.cc logger.o $(LDFLAGS) -o $(FILE) -Wall -O3

logger.o:	logger.h logger.cc
		$(CXX) $(FLAGS) logger.cc -c -Wall -O3

clean:
		rm -rf *.o *~ $(FILE) logger.o

