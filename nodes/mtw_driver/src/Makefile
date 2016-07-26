XSENS_INSTALL_PREFIX?=/usr/local/xsens
BASIC_CFLAGS := -g -Wall -Wextra
INCLUDE:=-I$(XSENS_INSTALL_PREFIX)/include
LFLAGS:=-lm -lxsensdeviceapi -lxstypes -lpthread -L$(XSENS_INSTALL_PREFIX)/lib -Wl,-rpath,$(XSENS_INSTALL_PREFIX)/lib

CFLAGS:=$(BASIC_CFLAGS) $(INCLUDE) $(CFLAGS)
CXXFLAGS:=$(BASIC_CFLAGS) $(INCLUDE) $(CXXFLAGS)

TARGETS:=example_mt_w_cpp mtw_node
all: $(TARGETS)

example_mt_w_cpp: example_mt_w_cpp.cpp.o console.cpp.o myxda.cpp.o conio.c.o
mtw_node: mtw_node.cpp.o console.cpp.o myxda.cpp.o conio.c.o
$(TARGETS):
	$(CXX) $(CFLAGS) $^ -o $@ $(LFLAGS)

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

clean:
	-$(RM) *.o *.d *.dpp $(TARGETS) $(PREBUILDARTIFACTS)
