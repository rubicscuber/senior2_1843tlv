CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
TARGET    = tm_visualizer
CONSOLE   = console_only
SRCDIR    = src
BUILDDIR  = build

# console_only.cpp has its own main(); build it as a separate executable
SRCS := $(filter-out $(SRCDIR)/console_only.cpp,$(wildcard $(SRCDIR)/*.cpp))
OBJS := $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(SRCS))
CONSOLE_OBJS := $(BUILDDIR)/console_only.o $(BUILDDIR)/frame_parser.o \
                $(BUILDDIR)/serial_port.o $(BUILDDIR)/cfg_parser.o

.PHONY: all clean

all: $(TARGET) $(CONSOLE)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(CONSOLE): $(CONSOLE_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

clean:
	rm -rf $(BUILDDIR) $(TARGET) $(CONSOLE)
