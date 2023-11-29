CC=gcc
CXX=g++
RM= /bin/rm -vf
ARCH=UNDEFINED
PWD=$(shell pwd)
CDR=$(shell pwd)
ECHO=echo

EDCFLAGS:=$(CFLAGS) -I include/ -I alliedcam/include -I rtd_adio/include -Wall -std=gnu11
EDLDFLAGS:=$(LDFLAGS) -lpthread -lm -L alliedcam/lib -lVmbC -L rtd_adio/lib -lrtd-aDIO
EDDEBUG:=$(DEBUG)

ifeq ($(ARCH),UNDEFINED)
	ARCH=$(shell uname -m)
endif

UNAME_S := $(shell uname -s)

EDCFLAGS+= -I include/ -I ./ -Wall -O2 -std=gnu11
CXXFLAGS:= -I alliedcam/include -I rtd_adio/include -I include/ -Wall -O2 -fpermissive -std=gnu++11 $(CXXFLAGS)
LIBS = -lpthread

ifeq ($(UNAME_S), Linux) #LINUX
	LIBS += -lGL `pkg-config --static --libs`
	CXXFLAGS += `pkg-config --cflags glfw3`
endif

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X"
	CXXFLAGS:= -arch $(ARCH) $(CXXFLAGS) `pkg-config --cflags`
	LIBS += -arch $(ARCH) `pkg-config --libs`
	LIBS += -L/usr/local/lib -L/opt/local/lib

	CXXFLAGS += -I/usr/local/include -I/opt/local/include `pkg-config --cflags`
	CFLAGS = $(CXXFLAGS)
endif

LIBS += -L alliedcam/lib -lVmbC -L rtd_adio/lib -lrtd-aDIO

all: CFLAGS+= -O2

GUITARGET=capture_server.out

all: clean $(GUITARGET)
	@$(ECHO)
	@$(ECHO)
	@$(ECHO) "Built for $(UNAME_S), execute \"LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):alliedcam/lib ./$(GUITARGET)\""
	@$(ECHO)
	@$(ECHO)
	LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):alliedcam/lib ./$(GUITARGET)

$(GUITARGET): alliedcam/liballiedcam.a rtd_adio/lib/librtd-aDIO.a
	$(CXX) -o $@ main.cpp stringhasher.cpp $(CXXFLAGS) $(LIBS)

alliedcam/liballiedcam.a:
	@$(ECHO) -n "Building alliedcam..."
	@cd $(PWD)/alliedcam && make liballiedcam.a && cd $(PWD)
	@$(ECHO) "done"

rtd_adio/lib/librtd-aDIO.a:
	@$(ECHO) -n "Building rtd_adio..."
	@cd $(PWD)/rtd_adio/lib && make && cd $(PWD)
	@$(ECHO) "done"

load:
	@$(ECHO) -n "Loading RTD aDIO driver..."
	@cd $(PWD)/rtd_adio/driver && make && make load && cd $(PWD)
	@$(ECHO) "done"

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	$(RM) $(GUITARGET)
	@cd $(PWD)/rtd_adio/lib && make clean && cd $(PWD)
	@cd $(PWD)/alliedcam && make clean && cd $(PWD)