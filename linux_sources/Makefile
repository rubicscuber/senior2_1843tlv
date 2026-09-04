CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
TARGET    = tm_visualizer
CONSOLE   = console_only
CONSOLEPI = console_only_Pi
SRCDIR    = src
BUILDDIR  = build

# console_only.cpp and console_only_Pi.cpp have their own main(); build them
# as separate executables
SRCS := $(filter-out $(SRCDIR)/console_only.cpp $(SRCDIR)/console_only_Pi.cpp,\
                     $(wildcard $(SRCDIR)/*.cpp))
OBJS := $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(SRCS))
SHARED_OBJS := $(BUILDDIR)/frame_parser.o $(BUILDDIR)/serial_port.o \
               $(BUILDDIR)/cfg_parser.o
CONSOLE_OBJS   := $(BUILDDIR)/console_only.o $(SHARED_OBJS)
CONSOLEPI_OBJS := $(BUILDDIR)/console_only_Pi.o $(SHARED_OBJS)

.PHONY: all clean

all: $(TARGET) $(CONSOLE) $(CONSOLEPI)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(CONSOLE): $(CONSOLE_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(CONSOLEPI): $(CONSOLEPI_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

clean:
	rm -rf $(BUILDDIR) $(TARGET) $(CONSOLE) $(CONSOLEPI)
