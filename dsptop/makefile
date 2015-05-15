#
# makefile for dsptop
#
# examples:
# make clean debug CC=your-favorite-gcc DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
# make clean release CC=your-favorite-gcc DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
#
# Note: if you have never built ncurses then do the following before you try to build dsptop:
# make ncurses
#
# To Use CROSS_COMPILE:
# make clean debug arm DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
# make clean release arm DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
# make all arm DESTDIR=your-destination-dir DEVICE=C66AK2Hxx
#
# Note: if you have never built ncurses then do the following before you try to build dsptop:
# make ncurses arm
# 

ifeq ($(findstring arm, $(MAKECMDGOALS)), arm) 
	CC= $(CROSS_COMPILE)gcc
endif
ifeq ($(findstring test, $(MAKECMDGOALS)), test) 
	TEST = -D TEST_MODE
else
	TEST =
endif

ifeq ($(findstring debug, $(MAKECMDGOALS)), debug)
        CFLAGS= -std=c99 -c -g -O -Wall -Wuninitialized $(INC_PATH) -D DEBUG -D $(DEVICE) -D _GNU_SOURCE
	OBJDIR= ./debug
        BUILD = debug
else 
        CFLAGS= -std=c99 -c -O2 -Wall -Wuninitialized $(INC_PATH) -D $(DEVICE) -D _GNU_SOURCE
	OBJDIR= ./release
        BUILD = release
endif

ifeq ($(findstring SUDO_BUILD, $(MAKECMDGOALS)), SUDO_BUILD) 
CFLAGS += -D SUDO_BUILD 
else 
CFLAGS += -D NO_PSC 
endif

LDFLAGS += -Wl,-rpath,\$$ORIGIN/.
LIBS= -lncurses -ltiulm
LIBS_PATH += -L ./ncurses/lib -L ./ulm/$(OBJDIR)/
INCLUDE_PATH +=  -I ./ncurses/include/ncurses -I ./ncurses/include -I ./ulm

INCLUDE_FILES = dsptop.h command.h error_handler.h error.h debug_log.h io.h signal_handler.h map.h etb_handler.h temp.h utility.h stp_decode_handler.h ulm_handler.h tiulm.h plot.h

.PHONY: clean debug release install arm ncurses ulm ncursesclean SUDO_BUILD

all: clean release install

#
# Declare.c and .h dependencies
#
dsptop.o: dsptop.c $(INCLUDE_FILES)
command.o: command.c $(INCLUDE_FILES)
error_handler.o : error_handler.c $(INCLUDE_FILES)
error.o : error.c $(INCLUDE_FILES)
debug_log.o : debug_log.c $(INCLUDE_FILES)
io.o : io.c $(INCLUDE_FILES)
signal_handler.o : signal_handler.c $(INCLUDE_FILES)
map.o : map.c $(INCLUDE_FILES)
etb_handler.o : etb_handler.c $(INCLUDE_FILES)
temp.o : temp.c $(INCLUDE_FILES)
TIETB.o : TIETB.c ETBAddr.h ETBInterface.h
utility.o : utility.c $(INCLUDE_FILES)
stpv2_dec.o : stpv2_dec.cpp stpv2_dec.h env_headers.h
stp_decode_handler.o : stp_decode_handler.c $(INCLUDE_FILES)
ulm_handler.o : ulm_handler.c $(INCLUDE_FILES)
plot.o : plot.c $(INCLUDE_FILES)

#
# Add objects
#

OBJECTS = $(addprefix $(OBJDIR)/, dsptop.o command.o error_handler.o error.o debug_log.o io.o signal_handler.o map.o temp.o etb_handler.o TIETB.o utility.o stpv2_dec.o stp_decode_handler.o ulm_handler.o plot.o)

$(OBJDIR)/%.o: %.c
	@echo "Compiling" $<
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDE_PATH) -o $@ $<

# Build .cpp files as C rather than default c++ - really for stpv2_dec only
$(OBJDIR)/%.o: %.cpp
	@echo "Compiling" $<
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -x c $(INCLUDE_PATH) -o $@ $<

dsptop: $(OBJECTS) 
	@echo "Building target" $@
	$(CC) $(LDFLAGS) $(LIBS_PATH) -o $(OBJDIR)/$@ $(OBJECTS) $(LIBS)

debug: ulm dsptop
	@echo "debug build complete"
	@echo  $(DEVICE) "version built"

release: ulm dsptop
	@echo "release build complete"
	@echo  $(DEVICE) "version built"

SUDO_BUILD:
	@echo "User must have sudo privilege"

arm:
	@echo "ARM version built"

test: dsptop
	@echo "Test version built - ETB file simulation"

install: man 
	install -m 755 -d ${DESTDIR}/usr/bin
	install -m 755 -d ${DESTDIR}/usr/share/man/man1
	-mv $(OBJDIR)/dsptop $(DESTDIR)/usr/bin/dsptop
	cp dsptop_sync $(DESTDIR)/usr/bin/dsptop_sync
	cp dsptop.1.gz $(DESTDIR)/usr/share/man/man1/dsptop.1.gz

clean:
	-rm $(OBJECTS)
	cd ulm; make clean

ncurses: ncursesclean
	cd ncurses; tar -zxf ncurses-5.9.tar.gz; cd ncurses-5.9; ./configure --target=arm-linux --with-shared --prefix $(CURDIR)/ncurses --without-tests --without-progs -without-cxx --host=$(NCURSES_HOST) --with-fallbacks=xterm,xterm-256color; make BUILD_CC=gcc CC=$(CC); make install.includes; make install.libs CC=$(CC)

ncursesclean:
	cd ncurses; rm -Rf bin; rm -Rf include; rm -Rf lib; rm -Rf man; rm -Rf share; rm -Rf ncurses-5.9;

ulm:
	make -C ulm $(BUILD) arm DEVICE=$(DEVICE)

man:
	rm -f dsptop.1.gz
	gzip -c dsptop.1 > dsptop.1.gz
 
