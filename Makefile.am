#define the output extension here
if !C4D_64BIT
OEXT=so
else C4D_64BIT
OEXT=so64
endif

noinst_PROGRAMS = $(c4d_bin)/plugins/AMa_ChamferMaker/AMa_ChamferMaker.$(OEXT)

AM_CXXFLAGS = -fPIC
AM_CFLAGS = -fPIC

__c4d_bin__plugins_AMa_ChamferMaker_AMa_ChamferMaker___OEXT__SOURCES = \
	./source/main.cpp \
	./source/AMa_ChamferMaker.cpp

if INTEL
INTELFLAGS = -limf -lirc
endif

__c4d_bin__plugins_AMa_ChamferMaker_AMa_ChamferMaker___OEXT__LDFLAGS = \
	-nodefaultlibs -shared -Bstatic \
	$(top_builddir)/work/bin/resource/_api_lib/libapi_lib.a \
	$(INTELFLAGS) -Wl,--version-script=$(c4d_bin)/resource/_api_lib/export_lx.map

INCLUDES =  \
	-I$(c4d_bin)/plugins/AMa_ChamferMaker/res \
	-I$(c4d_bin)/plugins/AMa_ChamferMaker/res/description

