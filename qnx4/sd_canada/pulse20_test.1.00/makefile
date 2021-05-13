# Makefile for pulse20_test
# =======================
# by Dieter ANdre
#
#

INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn \
	-I$(IPATH)/radarqnx4 \
	-I$(USR_IPATH)/radarqnx4/ops \
	-I$(USR_IPATH)/radarqnx4/site.$(SD_RADARCODE)

OBJS = pulse20_test.o
SRC=pulse20_test.c
OUTPUT = $(USR_BINPATH)/pulse20_test
SUDO = 1 
LIBS=-lsite.${SD_RADARCODE}.1 -lops.1 -lradar.1 -lerrlog.1 \
      -lrs.1 -lfreq.1 -liqcopy.1 -lacf.1 -lshmem.1 -ltcpipmsg.1 -lrawfeed.1 \
      -ltsg.1 -ltaskid.1 -lrmsgsnd.1 -lrtimer.1 -lrtime.1 -lrmath.1 -lopt.1 
include $(SITELIB).${SD_RADARCODE}
include $(MAKEBIN)






  









  


 


