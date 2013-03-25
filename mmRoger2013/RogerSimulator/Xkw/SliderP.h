#ifndef _SliderP_h
#define _SliderP_h

#include "Slider.h"

typedef struct {
	/* public */
	Pixel		foreground;	/* slider foreground */
	Pixel		fg_thumb;	/* thumb foreground color */
	XtOrientation	orientation;	/* horizontal or vertical */
	XtCallbackList	valueProc;	/* pass value by ref */
	Pixmap		thumb;		/* thumb color */
	Cursor		upCursor;	/* scroll up cursor */
	Cursor		downCursor;	/* scroll down cursor */
	Cursor		leftCursor;	/* scroll left cursor */
	Cursor		rightCursor;	/* scroll right cursor */
	Cursor		verCursor;	/* scroll vertical cursor */
	Cursor		horCursor;	/* scroll horizontal cursor */
	char		*label;		/* slider label */
	Double		start;		/* What percent above the win's top */
	Double		shown;		/* What percent is shown in the win */
	Double		min;		/* slider min value */
	Double		max;		/* slider max value */
	Double		value;		/* slider actual value */
	Double		inc;		/* slider step inc */
	Dimension	length;		/* either height or width */
	Dimension	thickness;	/* either width or height */
	Dimension	min_thumb;	/* minium size for the thumb. */
	Boolean		notify_cont;	/* notify continuous */
	Boolean		notify_mid;	/* notify continuous */
	Boolean		int_slider;	/* integer slider */
	Position	attach_thumb;	/* attach thumb to either end or self*/
	int             int_value;

	 /* private */
	Position	x;		/* slider rectangle x position */
	Position	y;		/* slider rectangle y position */
	Cursor		inactiveCursor; /* The normal cursor for slider */
	char		direction;	/* a scroll has started; which dir */
	GC		gc;		/* a (shared) gc */
	Position	startLoc;	/* Pixel that corresponds to start */
	Dimension	shownLength;	/* Num pixels corresponding to shown */
	XFontStruct	*font;		/* slider font */
} SliderPart;

typedef struct _SliderRec {
	CorePart	core;
	SliderPart	slider;
} SliderRec;

typedef struct {int empty;} SliderClassPart;

typedef struct _SliderClassRec {
	CoreClassPart	core_class;
	SliderClassPart	slider_class;
} SliderClassRec;

extern SliderClassRec	sliderClassRec;

#endif /* _SliderP_h */
