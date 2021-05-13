# Makefile for test_radar_dds
# =======================
# by DAndre
#
#

INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn \
	-I$(IPATH)/radarqnx4 \
	-I$(USR_IPATH)/radarqnx4/ops \
	-I$(USR_IPATH)/radarqnx4/site.$(SD_RADARCODE)

OBJS=test_radar_dds.o
SRC=test_radar_dds.c

OUTPUT = $(USR_BINPATH)/test_radar_dds
SUDO = 1 
LIBS=-lsite.${SD_RADARCODE}.1 -lops.1 -lfitacf.1 -lradar.1 -lerrlog.1 \
      -lrs.1 -lfreq.1 -liqcopy.1 -lacf.1 -lshmem.1 -ltcpipmsg.1 -lrawfeed.1 \
      -ltsg.1 -ltaskid.1 -lrmsgsnd.1 -lrtimer.1 -lrtime.1 -lrmath.1 -lopt.1 
include $(SITELIB).${SD_RADARCODE}
include $(MAKEBIN)






  









  


 


