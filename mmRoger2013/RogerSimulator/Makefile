# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

ROGERINCDIR = ./include

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 
EDLAB_XINCDIR = /usr/include/X11

XAWLIB = -lXaw
XMULIB = -lXmu
XTOOLLIB = -lXt
XLIB = -lX11
XEXTLIB = -lXext
MATHLIB = -lm

LIBS =  -L$(XLIBDIR) $(XAWLIB) $(XMULIB) $(XTOOLLIB) $(XLIB) $(XEXTLIB) \
	$(MATHLIB)

RM = rm -f
CC = gcc
#CCFLAGS = -c -O $(OPT) -I. -I$(XINCDIR)
CCFLAGS = -c -g -I. -I$(XINCDIR) -I$(EDLAB_XINCDIR) -I$(ROGERINCDIR) 

.SUFFIXES:	.c	.o

.c.o:	
	$(CC) $(CCFLAGS) $<

############## do not change above this line ####################

ROGERLIB = lib/simulator.a

OFILES1 = xrobot.o \
         mobile_base.o \
         arm.o \
	 eye.o \
         object.o \
	 4D_math.o

XKWOFILES = Xkw/Canvas.o Xkw/Slider.o Xkw/Xkw.o

HFILES = Roger.h simulate.h control.h modes.h

all:  subdirs  $(OFILES1) $(ROGERLIB)

subdirs:
	cd ./Xkw; make; cd ..; \

#$(ROGERLIB):	$(OFILES1)
#	$(CC) -o $@ $(OFILES1) $(XKWOFILES) $(LIBS)

$(ROGERLIB):	$(OFILES1) $(XKWOFILES)
	ar r $(ROGERLIB) $(OFILES1) $(XKWOFILES)

clean:
	$(RM) $(OFILES1) $(XKWOFILES) $(ROGERLIB) *~


#mobile_base.o:	Roger.h simulate.h control.h 

#arm.o:	Roger.h simulate.h control.h

#eye.o:	Roger.h simulate.h control.h

#object.o:	Roger.h simulate.h control.h

#xrobot.o:	Xkw/Xkw.h Roger.h simulate.h control.h

#tele_interface.o:	Roger.h	simulate.h control.h modes.h


