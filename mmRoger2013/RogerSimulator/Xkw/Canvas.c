/*
** Canvas.c
**
*/

# include <stdio.h>
# include <math.h>

# include <X11/Xos.h>
# include <X11/IntrinsicP.h>
# include <X11/StringDefs.h>
# include <X11/Xmu/Converters.h>

# include "CanvasP.h"

static char defaultTranslations[] =
	"<KeyPress>:		Notify()\n\
	<KeyRelease>:		Notify()\n\
	<ButtonPress>:		Notify()\n\
	<ButtonRelease>:	Notify()\n\
	<MotionNotify>:		Notify()\n\
	<ResizeRequest>:	Notify()\n\
	<ConfigureNotify>:	Notify()\n\
	<VisibilityNotify>:	Notify()\n\
	<EnterNotify>:		Notify()\n\
	<LeaveNotify>:		Notify()\n\
	<Expose>:		Notify()";

#define offset(field)	XtOffset(CanvasWidget,canvas.field)
#define goffset(field)	XtOffset(Widget,core.field)

static XtResource resources[] = {
	{XtNcallback, XtCCallback, XtRCallback, sizeof(XtPointer),
		offset(callbacks), XtRCallback, (XtPointer)NULL},
	{XtNwidth, XtCWidth, XtRDimension, sizeof(Dimension),
		goffset(width), XtRString, "512"},
	{XtNheight, XtCHeight, XtRDimension, sizeof(Dimension),
		goffset(height), XtRString, "512"},
	{XtNforeground, XtCForeground, XtRPixel, sizeof(Pixel),
		offset(foreground), XtRString, "Black"},
	{XtNbackground, XtCBackground, XtRPixel, sizeof (Pixel),
		offset(background), XtRString, "White"},
	{XtNbackingStore, XtCBackingStore, XtRBackingStore, sizeof (int),
		offset (backing_store), XtRString, "default"},
	{XtNcursor, XtCCursor, XtRCursor, sizeof(Cursor),
		offset(cursor), XtRString, "tcross"},
	{XtNlabel, XtCLabel, XtRString, sizeof(String),
	        offset(label), XtRString, NULL},
};

#undef offset
#undef goffset

static void Initialize(), Realize(), Destroy();
static void Redisplay(), Resize (), Notify();

static Boolean SetValues();
static int repaint_window();
static void ClassInitialize();

static XtActionsRec actionsList[] =
{
	{"Notify",	Notify},
};

CanvasClassRec canvasClassRec = {
	{ /* core fields */
	/* superclass		*/	&widgetClassRec,
	/* class_name		*/	"Canvas",
	/* size			*/	sizeof(CanvasRec),
	/* class_initialize	*/	ClassInitialize,
	/* class_part_initialize*/	NULL,
	/* class_inited		*/	FALSE,
	/* initialize		*/	Initialize,
	/* initialize_hook	*/	NULL,
	/* realize		*/	Realize,
	/* actions		*/	actionsList,
	/* num_actions		*/	XtNumber(actionsList),
	/* resources		*/	resources,
	/* num_resources	*/	XtNumber(resources),
	/* xrm_class		*/	(int) NULL,
	/* compress_motion	*/	TRUE,
	/* compress_exposure	*/	TRUE,
	/* compress_enterleave	*/	TRUE,
	/* visible_interest	*/	FALSE,
	/* destroy		*/	Destroy,
	/* resize		*/	Resize,
	/* expose		*/	Redisplay,
	/* set_values		*/	SetValues,
	/* set_values_hook	*/	NULL,
	/* set_values_almost	*/	NULL,
	/* get_values_hook	*/	NULL,
	/* accept_focus		*/	NULL,
	/* version		*/	XtVersion,
	/* callback_private	*/	NULL,
	/* tm_table		*/	defaultTranslations,
	/* query_geometry	*/	XtInheritQueryGeometry,
	}
};

static void ClassInitialize()
{
	XtAddConverter( XtRString, XtRBackingStore, XmuCvtStringToBackingStore,
		NULL, 0 );
}

WidgetClass canvasWidgetClass = (WidgetClass) &canvasClassRec;

/* ARGSUSED */
static void Initialize (greq, gnew)
Widget greq, gnew;
{
	CanvasWidget	w = (CanvasWidget)gnew;
	XtGCMask	valuemask;
	XGCValues	myXGCV;

	myXGCV.foreground = w->canvas.foreground;
	myXGCV.background = w->core.background_pixel;
	valuemask = GCForeground | GCBackground;
	w->canvas.gc = XtGetGC(gnew, valuemask, &myXGCV);
}

static void Resize (gw)
Widget	gw;
{
}

static void Realize (gw, valueMask, attrs)
Widget gw;
XtValueMask *valueMask;
XSetWindowAttributes *attrs;
{
	CanvasWidget	w = (CanvasWidget)gw;

	if (w->canvas.backing_store != Always + WhenMapped + NotUseful)
	{
	 	attrs->backing_store = w->canvas.backing_store;
		*valueMask |= CWBackingStore;
	}

	attrs->cursor = w->canvas.cursor; 
	*valueMask |= CWCursor;

	XtCreateWindow( gw, (unsigned)InputOutput, (Visual *)CopyFromParent,
		     *valueMask, attrs );
	Resize (gw);
}

static void Destroy (gw)
Widget gw;
{
	CanvasWidget w = (CanvasWidget)gw;

	XtDestroyGC (w->canvas.gc);
}

/* ARGSUSED */
static void Redisplay(w, event, region)
Widget w;
XEvent *event;
Region region;
{
}

static repaint_window (w)
CanvasWidget	w;
{
}
	
/* ARGSUSED */
static Boolean SetValues (current, request, new)
Widget current, request, new;
{
}

static void Notify(w,event,params,num_params)
Widget w;
XEvent *event;
String *params;         /* unused */
Cardinal *num_params;   /* unused */
{
	CanvasWidget gw = (CanvasWidget)w;

	XtCallCallbackList( (Widget) gw, gw->canvas.callbacks, event);
}

