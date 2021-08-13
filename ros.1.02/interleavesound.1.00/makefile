# Makefile for interleavesound
# ============================
# by E.G. Thomas
#
#

INCLUDE=-I$(IPATH)/general -I$(IPATH)/superdarn \
        -I$(IPATH)/superdarn/fit -I$(IPATH)/radar \
        -I$(USR_IPATH)/ops -I$(USR_IPATH)/site.$(SD_RADARCODE)

OBJS = interleavesound.o
SRC = interleavesound.c
SUDO=1
VERSION=1
OUTPUT = $(USR_BINPATH)/interleavesound
LIBS= -l$(SD_RADARCODE).site.1 -lops.1 -lrs.1 -lfclr.1 -lint.1 -lfreq.1 -lmsg.1 -lsys.1 -lad.1 -ldio.1 -ltsg.1 -lraw.1 -lradar.1 -lfitacf.1 -lgen.1 
include $(MAKEBIN)

