TARGET=libxscommon.a
CPP_FILES := $(wildcard *.cpp)
C_FILES := $(wildcard *.c)
OBJECTS := $(CPP_FILES:.cpp=.cpp.o) $(C_FILES:.c=.c.o)
CFLAGS+= -fPIC -I../ -include xscommon_config.h -include xstypes/xsens_compat.h
CXXFLAGS+= -std=c++11 -I../ -include xscommon_config.h -include xstypes/xsens_compat.h

all: $(TARGET)

-include $(FILES:.cpp=.dpp)

%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@
	@$(CXX) -MM $(CXXFLAGS) $< > $*.dpp
	@mv -f $*.dpp $*.dpp.tmp
	@sed -e 's|.*:|$*.cpp.o:|' < $*.dpp.tmp > $*.dpp
	@sed -e 's/.*://' -e 's/\\$$//' < $*.dpp.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.dpp
	@rm -f $*.dpp.tmp

-include $(FILES:.c=.d)

%.c.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@
	@$(CC) -MM $(CFLAGS) $< > $*.d
	@mv -f $*.d $*.d.tmp
	@sed -e 's|.*:|$*.c.o:|' < $*.d.tmp > $*.d
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

$(TARGET): $(OBJECTS)
	ar rcs $@ $^

clean:
	-$(RM) $(TARGET)
	-$(RM) *.o *.d *.dpp
