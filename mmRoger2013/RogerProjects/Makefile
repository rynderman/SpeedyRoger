# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

ROGERINCDIR = ./include

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 
EDLAB_XINCDIR = /usr/include/X11
EDLAB_XLIBDIR = /usr/lib/i386-linux-gnu


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
CCFLAGS = -c -g -I. -I$(XINCDIR) -I$(EDLAB_XINCDIR) -I$(ROGERINCDIR)

.SUFFIXES:	.c	.o

.c.o:	
	$(CC) $(CCFLAGS) $<

############## do not change above this line ####################

PROG = x

PROJECTOFILES = project1/MotorUnits.o \
		project2/Kinematics.o \
		project3/project3.o \
		project3/sampling.o \
                project3/vision.o \
		project4/project4.o \
		project5/project5.o \
		project6/project6.o  

#HFILES = Roger.h simulate.h $(ROGERINCDIR)/control.h

all:  subdirs  $(PROG)

subdirs:
	cd project1; make; cd ..; \
	cd project2; make; cd ..; \
	cd project3; make; cd ..; \
	cd project4; make; cd ..; \
	cd project5; make; cd ..; \
	cd project6; make; cd ..;

$(PROG): $(PROJECTOFILES) 
	$(CC) -o $@ lib/simulator.a $(LIBS) $^

clean:
	$(RM) $(PROJECTOFILES) $(PROG) *~


