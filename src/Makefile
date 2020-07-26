IDIR =/opt/mraa/include
CC=gcc
CPP=g++
CFLAGS=-I$(IDIR)
CPPFLAGS=$(CFLAGS) -std=gnu++11
ODIR=obj
LDIR=-L/opt/mraa/lib

LIBS=-lmraa -lmosquitto

#_DEPS = hellomake.h
#DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))
DEPS= $(wildcard *.h)
SRC=$(wildcard *.cpp)

_OBJ = $(SRC:.cpp=.o)
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.c $(DEPS)
	@mkdir -p $(ODIR)
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o: %.cpp $(DEPS)
	@mkdir -p $(ODIR)
	$(CPP) -c -o $@ $< $(CPPFLAGS)

hexapod: $(OBJ)
	g++ -o $@ $^ $(CPFLAGS) $(LDIR) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o core hexapod
