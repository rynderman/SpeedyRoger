#ifndef _XCanvasP_h
#define _XCanvasP_h

#include "Canvas.h"
#include <X11/CoreP.h>

#define SEG_BUFF_SIZE		128

/* New fields for the canvas widget instance record */
typedef struct {
	Pixel		foreground;	/* foreground pixel */
	Pixel		background;	/* background pixel */
	GC		gc;		/* pointer to GraphicsContext */
	int		backing_store;	/* backing store variety */
	XtCallbackList	callbacks;
	Cursor		cursor;
	char            *label;
} CanvasPart;

/* Full instance record declaration */
typedef struct _CanvasRec {
	CorePart	core;
	CanvasPart	canvas;
} CanvasRec;

/* New fields for the Canvas widget class record */
typedef struct {int dummy;} CanvasClassPart;

/* Full class record declaration. */
typedef struct _CanvasClassRec {
	CoreClassPart	core_class;
	CanvasClassPart	canvas_class;
} CanvasClassRec;

/* Class pointer. */
extern CanvasClassRec canvasClassRec;

#endif /* _CanvasP_h */
