#
# makefile for ulm
#
# examples:
#
# make clean debug CC=your-favorite-gcc DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
#
# make clean release CC=your-favorite-gcc DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx

#make for OpenCL Linux component
#
#make clean release arm DEVICE=C66AK2Hxx XPORT_ONLY

# To make with arm CROSS_COMPILE:
# make clean debug arm DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
#
# make clean release arm DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
#
# make all arm DESTDIR=your-destination-dir DEVICE=C66AK2Hxx
#   
# To make with dsp CROSS_COMPILE:
# make clean debug dsp DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
# make clean release dsp DESTDIR=your-destination-dir install DEVICE=C66AK2Hxx
# make all dsp DESTDIR=your-destination-dir DEVICE=C66AK2Hxx 

# To make dsp with CROSS_COMPILE:
# make clean debug dsp DEVICE=C66AK2Hxx

CFLAGS = -c $(INC_PATH) -D $(DEVICE) -D _GNU_SOURCE -D LIB_BUILD

LFLAGS =
LIBS=
LIBS_PATH=
INCLUDE_PATH = -I ../

INCLUDE_FILES = tiulm.h stm.h map.h version.h

ifeq ($(findstring debug, $(MAKECMDGOALS)), debug) 
     CFLAGS += -g -D DEBUG
     OBJDIR= ./debug
else 
     CFLAGS += -O2
     OBJDIR= ./release
endif

ifeq ($(findstring arm, $(MAKECMDGOALS)), arm) 
    CC= $(CROSS_COMPILE)gcc
    AR= $(CROSS_COMPILE)ar
    CFLAGS += -std=c99 -Wall -D _GNU_SOURCE -fPIC
ifeq ($(findstring XPORT_ONLY, $(MAKECMDGOALS)), XPORT_ONLY)
    CFLAGS += -D XPORT_ONLY
endif
ifeq ($(findstring SUDO_BUILD, $(MAKECMDGOALS)), SUDO_BUILD) 
    CFLAGS += -D SUDO_BUILD 
endif
    ARFLAGS= rcs
    OBJECTS = $(addprefix $(OBJDIR)/, ulm.o map.o)
endif

#todo make a .lib from cl6x
ifeq ($(findstring dsp, $(MAKECMDGOALS)), dsp) 
  CC = cl6x
  AR = ar6x
  ARFLAGS= qru
  CFLAGS += -mv6600 --abi=eabi --gcc --mem_model:const=data --mem_model:data=far
  OBJECTS = $(addprefix $(OBJDIR)/, ulm.obj)
  ifeq ($(C6X_C_DIR),)
      $(error To compile for dsp set the C6X_C_DIR environment variable to point at the C6000 Codegen Tool Installation)
  endif
endif

.PHONY: clean debug release install arm XPORT_ONLY SUDO_BUILD

all: clean release install

$(OBJDIR)/%.o: %.c $(INCLUDE_FILES)
	@echo "Compiling" $<
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDE_PATH) -o $@ $<

$(OBJDIR)/%.obj: %.c $(INCLUDE_FILES)
	@echo "Compiling" $<
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDE_PATH) -fe $@ $<

libtiulm.a libtiulm.ae66: $(OBJECTS)
	@echo "Building target" $@
	$(AR) $(ARFLAGS) $(OBJDIR)/$@ $(OBJECTS)

debug:
	@echo "Debug build"
	@echo  $(DEVICE) "version built"

release:
	@echo "Release build"
	@echo  $(DEVICE) "version built"

arm: libtiulm.a
	@echo "ARM version build"

dsp: libtiulm.ae66
	@echo "DSP version build"

install: 
ifeq ($(findstring arm, $(MAKECMDGOALS)), arm) 
	install -m 755 -d ${DESTDIR}/usr/lib
	install -m 755 -d ${DESTDIR}/usr/include
	mv $(OBJDIR)/libtiulm.a $(DESTDIR)/usr/lib
	cp tiulm.h $(DESTDIR)/usr/include
endif
ifeq ($(findstring dsp, $(MAKECMDGOALS)), dsp)
	install -m 755 -d ${DESTDIR}/usr/share/ti/ulm
	mv $(OBJDIR)/libtiulm.ae66 $(DESTDIR)/usr/share/ti/ulm
	cp tiulm.h ${DESTDIR}/usr/share/ti/ulm
endif 

clean:
	@rm -f ./release/*.obj ./release/*.o ./debug/*.obj ./debug/*.o ./release/*.a ./debug/*.a ./release/*.lib ./debug/*.lib ./release/*.ae66 ./debug/*.ae66

XPORT_ONLY:
	@echo "XPORT_ONLY version - only STM transport is mapped by ulm_config()"

SUDO_BUILD:
	@echo "User must have sudo privilege"

