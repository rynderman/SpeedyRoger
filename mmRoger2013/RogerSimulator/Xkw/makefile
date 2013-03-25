# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

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

LIBS =  -L$(XLIBDIR) -L$(EDLAB_XLIBDIR) $(XAWLIB) $(XMULIB) $(XTOOLLIB) $(XLIB) $(XEXTLIB) \
        $(MATHLIB)

RM = rm -f
CC = gcc
#CCFLAGS = -c -g $(OPT) -I. -I$(XINCDIR)
CCFLAGS = -c -g $(OPT) -I. -I$(EDLAB_XINCDIR) -I$(XINCDIR)

.SUFFIXES:      .c      .o

.c.o:
	$(CC) $(CCFLAGS) $<

############## do not change above this line ####################

PROG = x

XKWOFILES = Canvas.o Xkw.o Slider.o

HFILES =


all:	$(PROG)

#$(PROG):	$(XKWOFILES)
#	$(CC) -o $@ $(XKWOFILES) $(LIBS)
$(PROG):	$(XKWOFILES)


clean:
	$(RM) $(XKWOFILES) $(PROG)

Canvas.o: Canvas.h
Slider.o: Slider.h
Xkw.o: Xkw.h
