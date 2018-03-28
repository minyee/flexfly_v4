TARGET := libflexfly.so 
SRC := link_stealing.cc flexfly_topology_simplified.cc

CXX :=    libsst++
CC :=     libsstcc
CXXFLAGS := -fPIC -std=c++11
CPPFLAGS := -I. -I$(HOME)/Programs/install/sst-core/clang/include -I$(HOME)/src/alglib-3.12/src 
#CPPFLAGS := -I. -I$(HOME)/install/sstcore-7.1.0/include
LIBDIR :=  
PREFIX := 
LDFLAGS :=  -Wl,-rpath,$(PREFIX)/lib

OBJ := $(SRC:.cc=.o) 
OBJ := $(OBJ:.cpp=.o)
OBJ := $(OBJ:.c=.o)

.PHONY: clean install 

all: $(TARGET)

$(TARGET): $(OBJ) 
	$(CXX) -o $@ $+ $(LDFLAGS) $(LIBS)  $(CXXFLAGS)

%.o: %.cc 
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

clean: 
	rm -f $(TARGET) $(OBJ) 

install: $(TARGET)
	cp $< $(PREFIX)/bin

