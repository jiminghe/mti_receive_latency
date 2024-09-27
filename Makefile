CXX = g++
CXXFLAGS = -std=c++11 -Wall
LDFLAGS = -lboost_system -lpthread

TARGET = mti_receive_block
SRCS = mti_receive_block.cpp

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS) $(LDFLAGS)

clean:
	rm -f $(TARGET)