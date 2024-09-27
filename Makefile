CXX = g++
CXXFLAGS = -std=c++11 -Wall
LDFLAGS = -lboost_system -lpthread

TARGET = mti_receive_nblock
SRCS = mti_receive_nblock.cpp

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS) $(LDFLAGS)

clean:
	rm -f $(TARGET)