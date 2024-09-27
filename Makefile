TARGETS:= mti_receive
OBJLIBS	= xspublic
INCLUDE=-I. -Ixspublic
CFLAGS=-g $(INCLUDE)
CXXFLAGS=-std=c++11 $(CFLAGS)
LFLAGS=-Lxspublic/xscontroller -Lxspublic/xscommon -Lxspublic/xstypes -lxscontroller -lxscommon -lxstypes -lpthread -lrt -ldl

.PHONY: $(OBJLIBS)
all : $(OBJLIBS) $(TARGETS)

xspublic :
	$(MAKE) -C xspublic $(MFLAGS)

mti_receive: example_mti_receive_data.cpp.o


$(TARGETS):
	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LFLAGS)

%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

clean :
	-$(RM) $(OBJECTS) $(TARGETS)
	-$(RM) *.o *.dpp *.d
	-$(MAKE) -C xspublic/xscontroller $(MFLAGS) clean
	-$(MAKE) -C xspublic/xscommon $(MFLAGS) clean
	-$(MAKE) -C xspublic/xstypes $(MFLAGS) clean
