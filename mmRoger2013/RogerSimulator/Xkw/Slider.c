/* Slider.c */

#include <math.h>
#include <X11/IntrinsicP.h>
#include <X11/StringDefs.h>

#include <X11/Xaw/XawInit.h>
#include "SliderP.h"

#define P		printf("%ld\n", __LINE__)
#define D(num)		printf("%ld: %lf\n", __LINE__, num);

/* Private definitions. */

static char defaultTranslations[] =
	"<Btn1Down>:	StartSlide(Backward) \n\
	<KeyPress>:	ZeroSlide() \n\
	<Btn2Down>:	StartSlide(Continuous) MoveThumb() \n\
	<Btn3Down>:	StartSlide(Forward) \n\
	<Btn2Motion>:	MoveThumb() NotifyThumb(Motion) \n\
	<Btn1Up>:	NotifySlide(Prop) NotifyThumb(Up) EndSlide() \n\
	<Btn2Up>:	NotifySlide(Prop) NotifyThumb(Up) EndSlide() \n\
	<Btn3Up>:	NotifySlide(Prop) NotifyThumb(Up) EndSlide()";

static double	value = 0.0;
static double	min = -1.0;
static double	max = 1.0;
static double	inc = -999;
int		font_height, font_width, len;
int		hlpos, vlpos;

#define Offset(field)	XtOffset(SliderWidget, field)

static XtResource resources[] = {
	{XtNlength, XtCLength, XtRDimension, sizeof(Dimension),
		Offset(slider.length), XtRImmediate, (caddr_t) 200},
	{XtNthickness, XtCThickness, XtRDimension, sizeof(Dimension),
		Offset(slider.thickness), XtRImmediate, (caddr_t) 15},
	{XtNorientation, XtCOrientation, XtROrientation, sizeof(XtOrientation),
		Offset(slider.orientation), XtRImmediate,
			(caddr_t) XtorientHorizontal},
	{XtNvalueProc, XtCCallback, XtRCallback, sizeof(caddr_t),
		Offset(slider.valueProc), XtRCallback, NULL},
	{XtNthumb, XtCThumb, XtRPixmap, sizeof(Pixmap),
		Offset(slider.thumb), XtRImmediate,
			(XtPointer) XtUnspecifiedPixmap},
	{XtNfont,  XtCFont, XtRFontStruct, sizeof(XFontStruct *),
		Offset(slider.font),XtRString, "XtDefaultFont"},
	{XtNforeground, XtCForeground, XtRPixel, sizeof(Pixel),
		Offset(slider.foreground), XtRString, XtDefaultForeground},
	{XtNforegroundThumb, XtCForeground, XtRPixel, sizeof(Pixel),
		Offset(slider.fg_thumb), XtRString, XtDefaultForeground},
	{XtNscrollVCursor, XtCCursor, XtRCursor, sizeof(Cursor),
		Offset(slider.verCursor), XtRString, "sb_v_double_arrow"},
	{XtNscrollHCursor, XtCCursor, XtRCursor, sizeof(Cursor),
		Offset(slider.horCursor), XtRString, "sb_h_double_arrow"},
	{XtNscrollUCursor, XtCCursor, XtRCursor, sizeof(Cursor),
		Offset(slider.upCursor), XtRString, "sb_up_arrow"},
	{XtNscrollDCursor, XtCCursor, XtRCursor, sizeof(Cursor),
		Offset(slider.downCursor), XtRString, "sb_down_arrow"},
	{XtNscrollLCursor, XtCCursor, XtRCursor, sizeof(Cursor),
		Offset(slider.leftCursor), XtRString, "sb_left_arrow"},
	{XtNscrollRCursor, XtCCursor, XtRCursor, sizeof(Cursor),
		Offset(slider.rightCursor), XtRString, "sb_right_arrow"},
	{XtNminimumThumb, XtCMinimumThumb, XtRDimension, sizeof(Dimension),
		Offset(slider.min_thumb), XtRImmediate, (caddr_t) 7},
	{XtNintValue, XtCValue, XtRInt, sizeof(int),
		Offset(slider.int_value), XtRImmediate, (caddr_t) 0},
	{XtNlabel, XtCLabel, XtRString, sizeof(String),
		Offset(slider.label), XtRString, NULL},
	{XtNshown, XtCValue, XtRDouble, sizeof(Double),
		Offset(slider.shown), XtRDouble, (caddr_t)&value},
	{XtNstartOfThumb, XtCValue, XtRDouble, sizeof(Double),
		Offset(slider.start), XtRDouble, (caddr_t)&value},
	{XtNvalue, XtCValue, XtRDouble, sizeof(Double),
		Offset(slider.value), XtRDouble, (caddr_t)&value},
	{XtNmin, XtCValue, XtRDouble, sizeof(Double),
		Offset(slider.min), XtRDouble, (caddr_t)&min},
	{XtNmax, XtCValue, XtRDouble, sizeof(Double),
		Offset(slider.max), XtRDouble, (caddr_t)&max},
	{XtNinc, XtCValue, XtRDouble, sizeof(Double),
		Offset(slider.inc), XtRDouble, (caddr_t)&inc},
	{XtNnotifyMid, XtCNotify, XtRBoolean, sizeof(Boolean),
		Offset(slider.notify_mid), XtRBoolean, (caddr_t)TRUE},
	{XtNnotifyCont, XtCNotify, XtRBoolean, sizeof(Boolean),
		Offset(slider.notify_cont), XtRBoolean, (caddr_t)FALSE},
	{XtNattachThumb, XtCAttachThumb, XtRPosition, sizeof(Position),
		Offset(slider.attach_thumb), XtRImmediate, (caddr_t) Self},
	{XtNintSlider, XtCNotify, XtRBoolean, sizeof(Boolean),
		Offset(slider.int_slider), XtRBoolean, (caddr_t)FALSE},
};

static void	ClassInitialize();
static void	Initialize();
static void	Destroy();
static void	Realize();
static void	Resize();
static void	Redisplay();
static Boolean	SetValues();

static void	StartSlide();
static void	MoveThumb();
static void	NotifyThumb();
static void	NotifySlide();
static void	EndSlide();
static void	ZeroSlide();

static XtActionsRec actions[] = {
	{"StartSlide",		StartSlide},
	{"MoveThumb",		MoveThumb},
	{"NotifyThumb",		NotifyThumb},
	{"NotifySlide",		NotifySlide},
	{"EndSlide",		EndSlide},
	{"ZeroSlide",		ZeroSlide},
	{NULL,			NULL}
};

SliderClassRec sliderClassRec = {
/* core fields */
    /* superclass       */      (WidgetClass) &widgetClassRec,
    /* class_name       */      "Slider",
    /* size             */      sizeof(SliderRec),
    /* class_initialize	*/	ClassInitialize,
    /* class_part_init  */	NULL,
    /* class_inited	*/	FALSE,
    /* initialize       */      Initialize,
    /* initialize_hook  */	NULL,
    /* realize          */      Realize,
    /* actions          */      actions,
    /* num_actions	*/	XtNumber(actions),
    /* resources        */      resources,
    /* num_resources    */      XtNumber(resources),
    /* xrm_class        */      NULLQUARK,
    /* compress_motion	*/	TRUE,
    /* compress_exposure*/	TRUE,
    /* compress_enterleave*/	TRUE,
    /* visible_interest */      FALSE,
    /* destroy          */      Destroy,
    /* resize           */      Resize,
    /* expose           */      Redisplay,
    /* set_values       */      SetValues,
    /* set_values_hook  */	NULL,
    /* set_values_almost */	XtInheritSetValuesAlmost,
    /* get_values_hook  */	NULL,
    /* accept_focus     */      NULL,
    /* version          */	XtVersion,
    /* callback_private */      NULL,
    /* tm_table         */      defaultTranslations,
    /* query_geometry	*/	XtInheritQueryGeometry,
    /* display_accelerator*/	XtInheritDisplayAccelerator,
    /* extension        */	NULL
};

WidgetClass sliderWidgetClass = (WidgetClass)&sliderClassRec;

#define NoButton	-1
#define PICKLENGTH(widget, x, y) \
	((widget->slider.orientation == XtorientHorizontal) ? x : y)
#define MIN(x,y)	((x) < (y) ? (x) : (y))
#define MAX(x,y)	((x) > (y) ? (x) : (y))

static void ClassInitialize()
{
	static XtConvertArgRec screenConvertArg[] = {
		{XtWidgetBaseOffset, (caddr_t) XtOffset(Widget, core.screen),
			sizeof(Screen *)}
		};

	XawInitializeWidgetSet();
	XtAddConverter( XtRString, XtROrientation, XmuCvtStringToOrientation,
			NULL, (Cardinal)0 );
	XtAddConverter( XtRString, XtRPixmap, XmuCvtStringToBitmap,
			screenConvertArg, XtNumber(screenConvertArg));
}

/*
** Make sure the first number is within the range specified by the other
** two numbers.
*/
static double DoubleInRange(num, small, big)
double num, small, big;
{
	return (num < small) ? small : ((num > big) ? big : num);
}


/*
** Fill the area specified by start and end with the given pattern.
*/
static double PercentLoc(w, x, y)
SliderWidget w;
int x, y;
{
	double   result;

	result = PICKLENGTH(w,
		(double) x/w->slider.length,
		(double) -y/w->slider.length);

	return DoubleInRange(result, 0.0, 1.0);
}

static FillArea(w, start, end, thumb)
SliderWidget w;
Position start, end;
int thumb;
{
	Dimension length = end-start;

	if (end < 0) return;

	XSetForeground(XtDisplay(w), w->slider.gc, w->slider.fg_thumb);
	switch(thumb)
	{
	case 1:	/* Fill the new Thumb location */
		if (w->slider.orientation == XtorientHorizontal) 
			XFillRectangle(XtDisplay(w), XtWindow(w), w->slider.gc,
				start, w->slider.y,
				length, w->slider.thickness-1);
	
		else	XFillRectangle(XtDisplay(w), XtWindow(w), w->slider.gc,
				w->slider.x, start,
				w->slider.thickness-1, length);

		break;

	case 0:	/* Clear the old Thumb location */
		if (w->slider.orientation == XtorientHorizontal) 
			XClearArea(XtDisplay(w), XtWindow(w),
				start, w->slider.y,
		       		length, w->slider.thickness-1, FALSE);
	
		else	XClearArea(XtDisplay(w), XtWindow(w),
				w->slider.x, start,
				w->slider.thickness-1, length, FALSE);
	}  
	XSetForeground(XtDisplay(w), w->slider.gc, w->slider.foreground);
}


/*
** Paint the thumb in the area specified by w->start and
** w->shown.  The old area is erased.  The painting and
** erasing is done cleverly so that no flickering will occur.
*/
static void PaintThumb( w )
SliderWidget w;
{
	Position oldstart, oldend, newstart, newend;

	oldstart = w->slider.startLoc;
	oldend = oldstart + w->slider.shownLength;

	if (w->slider.orientation == XtorientHorizontal)
	{
		if(w->slider.attach_thumb == Self)
		{
		newstart = w->slider.x + w->slider.length * w->slider.start;
		newend = newstart + (int)(w->slider.length * w->slider.shown);
		if (newend < newstart + w->slider.min_thumb)
			newend = newstart + w->slider.min_thumb;
		}
		else if(w->slider.attach_thumb == Min)
		{
		newstart = w->slider.x;
		newend = w->slider.x + w->slider.min_thumb
			+ w->slider.length * w->slider.start;
		}
		else if(w->slider.attach_thumb == Max)
		{
		newstart = w->slider.x + w->slider.length * w->slider.start;
		newend = w->slider.x + w->slider.length + w->slider.min_thumb;
		}
	}
	else
	{
		if(w->slider.attach_thumb == Self)
		{
		newstart = w->slider.y - w->slider.length * w->slider.start;
		newend = newstart + (int)(w->slider.length * w->slider.shown);
		if (newend < newstart + w->slider.min_thumb)
			newend = newstart + w->slider.min_thumb;
		}
		else if(w->slider.attach_thumb == Min)
		{
		newstart = w->slider.y - w->slider.length * w->slider.start;
		newend = w->slider.y + w->slider.min_thumb;
		}
		else if(w->slider.attach_thumb == Max)
		{
		newstart = w->slider.y - w->slider.length;
		newend = w->slider.y + w->slider.min_thumb
			- w->slider.length * w->slider.start;
		}
	}

	w->slider.shownLength = newend - newstart;
	w->slider.startLoc = newstart;

	if (XtIsRealized((Widget)w))
	{
	if(newstart < oldstart) FillArea(w, newstart, MIN(newend, oldstart), 1);
	if(newstart > oldstart) FillArea(w, oldstart, MIN(newstart, oldend), 0);
	if(newend < oldend) FillArea(w, MAX(newend, oldstart), oldend, 0);
	if(newend > oldend) FillArea(w, MAX(newstart, oldend), newend, 1);
	}
}

static void PaintValue( w )
SliderWidget w;
{
	char label[128];
	char fmt[16];
	double log10();

	if(w->slider.label[0] != '\0') {
	XSetFillStyle(XtDisplay(w), w->slider.gc, FillSolid);
	if(w->slider.int_slider == TRUE) {
	  char fmt[16];
	  sprintf(fmt, "%%0%dd", 1+(int)log10(fabs(w->slider.max)));
	  sprintf(label, fmt, (int) w->slider.value);
	}
	else 
	  sprintf(label, "%4.2f", w->slider.value);
	if (w->slider.orientation == XtorientHorizontal)
	{
		XClearArea(XtDisplay(w), XtWindow(w),
			w->slider.x-140, w->slider.y,
			50, w->core.height, FALSE);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			w->slider.x-140, hlpos,
			label, strlen(label));
	}
	else
	{
		XClearArea(XtDisplay(w), XtWindow(w),
			vlpos+font_width, 3*font_height,
			12*font_width, 2*font_height, FALSE);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			vlpos+2*font_width, 4*font_height,
			label, strlen(label));
	}
/*
	XSetFillStyle(XtDisplay(w), w->slider.gc, FillOpaqueStippled);
*/
	}
}

/*
**	Function Name: Destroy
**	Description: Called as the slider is going away...
**	Arguments: w - the slider.
**	Returns: nonw
*/

static void Destroy(w)
Widget w;
{
	SliderWidget sbw = (SliderWidget) w;
    
	XtReleaseGC(w, sbw->slider.gc);
}

/*
**	Function Name: CreateGC
**	Description: Creates the GC.
**	Arguments: w - the slider widget.
**	Returns: none. 
*/

static void CreateGC(w)
Widget w;
{
	SliderWidget	sbw = (SliderWidget) w;
	XGCValues	gcValues;
	XtGCMask	mask;
	unsigned	int depth = 1;
	XFontStruct	*font;

	if (sbw->slider.thumb == XtUnspecifiedPixmap)
	{
		sbw->slider.thumb = XmuCreateStippledPixmap (XtScreen(w), 
			(Pixel) 1, (Pixel) 0, depth);
	}
	else if (sbw->slider.thumb != None)
	{
		Window root;
		int x, y;
		unsigned int width, height, bw;

		if (XGetGeometry(XtDisplay(w), sbw->slider.thumb, &root, &x, &y,
			&width, &height, &bw, &depth) == 0)
		{
			XtAppError(XtWidgetToApplicationContext(w),
		"Slider Widget: Could not get geometry of thumb pixmap.");
		}
	}

/*
	sbw->slider.font = XLoadQueryFont(XtDisplay(sbw),
		"-*-fixed-*-*-*-*-*-*-*-*-*-*-*-*");
*/
	gcValues.foreground = sbw->slider.foreground;
	gcValues.background = sbw->core.background_pixel;
	gcValues.font = sbw->slider.font->fid;
	mask = GCForeground | GCBackground | GCFont;

/*
	if (sbw->slider.thumb != None)
	{
		if (depth == 1)
		{
			gcValues.fill_style = FillOpaqueStippled;
			gcValues.stipple = sbw->slider.thumb;
			mask |= GCFillStyle | GCStipple;
		}
		else
		{
			gcValues.fill_style = FillTiled;
			gcValues.tile = sbw->slider.thumb;
			mask |= GCFillStyle | GCTile;
		}
	}
*/
	sbw->slider.gc = XtGetGC( w, mask, &gcValues);
	XSetFillStyle(XtDisplay(w), sbw->slider.gc, FillSolid);
}

static void SetDimensions(w)
SliderWidget w;
{
	int	len;
	char	str[128];
	register XFontStruct        *fs = w->slider.font;

	len = strlen(w->slider.label);
	font_height = fs->max_bounds.ascent + fs->max_bounds.descent;
	font_width  = fs->max_bounds.rbearing - fs->max_bounds.lbearing;

	if (w->slider.orientation == XtorientHorizontal)
	{
        	if(w->slider.label[0] == '\0') {
		w->slider.x = 0;  
		w->slider.y = 0;  
		w->slider.startLoc = w->slider.x;
		if(w->core.width == 0)
			w->core.width =
			w->slider.x + w->slider.length + w->slider.min_thumb;
		else
		        w->slider.length =
			w->core.width - w->slider.x - w->slider.min_thumb;
		if(w->core.height == 0)
			w->core.height = w->slider.thickness;
		else
		        w->slider.thickness = w->core.height;
		}
		else {
		w->slider.x = len*font_width+200;  
		w->slider.y = 5;
		w->slider.startLoc = w->slider.x;
		if(w->core.width == 0)
			w->core.width = w->slider.x + w->slider.length + 70;
		else
		        w->slider.length = w->core.width - w->slider.x - 70;
		if(w->core.height == 0)
			w->core.height = w->slider.thickness + 10;
		else
		        w->slider.thickness = w->core.height - 10;
		hlpos = 5+font_height;  
		hlpos = w->core.height/2+font_height/2;  
/*
		printf("x = %ld\n", w->slider.x);
		printf("y = %ld\n", w->slider.y);
		printf("w = %ld\n", w->core.width);
		printf("h = %ld\n", w->core.height);
		printf("t = %ld\n", w->slider.thickness);
*/
		}
	}
	else
	{
                if(w->slider.label[0] == '\0') {
		if(w->core.width == 0) w->core.width = w->slider.thickness;
		else w->slider.thickness = w->core.width;
		if(w->core.height == 0)
			w->core.height = w->slider.length + w->slider.min_thumb;
		else w->slider.length = w->core.height - w->slider.min_thumb;
		w->slider.y = w->slider.length;  
		w->slider.x = 0; 
		w->slider.startLoc = w->slider.y;
		}
		else {
		if(w->core.width == 0)
			w->core.width = len*font_width + 50;
		else
		        w->slider.thickness = w->core.width - 60;
		if(w->core.height == 0)
			w->core.height = w->slider.y + 40;
		else
		        w->slider.length = w->core.height - 7*font_height - 40;
		w->slider.y = 7*font_height + w->slider.length;  
		w->slider.x = len*font_width - w->slider.thickness/2; 
		w->slider.startLoc = w->slider.y;
		vlpos = 5;
		}
	}
	w->slider.direction = 0;
	w->slider.shownLength = w->slider.min_thumb;
	w->slider.start = (w->slider.value-w->slider.min)
		/(w->slider.max-w->slider.min);
}

/* ARGSUSED */
static void Initialize( request, new )
Widget request;		/* what the client asked for */
Widget new;		/* what we're going to give him */
{
	SliderWidget w = (SliderWidget) new;

	CreateGC(new);
	SetDimensions(new);
}

static void Realize( gw, valueMask, attributes )
Widget gw;
Mask *valueMask;
XSetWindowAttributes *attributes;
{
	SliderWidget w = (SliderWidget) gw;

	w->slider.inactiveCursor = (w->slider.orientation == XtorientVertical)
		? w->slider.verCursor : w->slider.horCursor;

	attributes->cursor = w->slider.inactiveCursor;
	*valueMask |= CWCursor;
    
	XtCreateWindow( gw, InputOutput, (Visual *)CopyFromParent,
		*valueMask, attributes );
}

/* ARGSUSED */
static Boolean SetValues( current, request, desired )
Widget current,		/* what I am */
       request,		/* what he wants me to be */
       desired;		/* what I will become */
{
	SliderWidget w = (SliderWidget) current;
	SliderWidget dw = (SliderWidget) desired;
	Boolean redraw = FALSE;

/*
** If these values are outside the acceptable range ignore them...
*/

	if (dw->slider.start < 0.0 || dw->slider.start > 1.0)
		dw->slider.start = w->slider.start;

	if (dw->slider.shown < 0.0 || dw->slider.shown > 1.0)
		dw->slider.shown = w->slider.shown;

/*
** Change colors and stuff...
*/

	if ( XtIsRealized (desired) )
	{
		if ( (w->slider.foreground != dw->slider.foreground) ||
		(w->core.background_pixel != dw->core.background_pixel) ||
		(w->slider.thumb != dw->slider.thumb) ) 
		{
			XtReleaseGC((Widget) w, (GC) w->slider.thumb);
			CreateGC( (Widget) dw);
			redraw = TRUE;
		}
		if (w->slider.start != dw->slider.start ||
		w->slider.shown != dw->slider.shown)
			redraw = TRUE;
	}

	return( redraw );
}

static void Resize( gw )
Widget gw;
{
/*
** ForgetGravity has taken care of background, but thumb may
** have to move as a result of the new size.
*/
	SetDimensions( (SliderWidget)gw );
	Redisplay( gw, (XEvent*)NULL, (Region)NULL );
}


/* ARGSUSED */
static void Redisplay( gw, event, region )
Widget gw;
XEvent *event;
Region region;
{
	SliderWidget w = (SliderWidget) gw;
	int x, y, i;
	unsigned int width, height;
	char labelstr[128], label[128];
	double	start;

	if (w->slider.orientation == XtorientHorizontal)
	{
		x = w->slider.startLoc;
		y = 1;
		width = w->slider.shownLength;
		height = w->slider.thickness;
	}
	else
	{
		x = 1;
		y = w->slider.startLoc;
		width = w->slider.thickness;
		height = w->slider.shownLength;
	}

	if ( (region == NULL) ||
	(XRectInRegion(region, x, y, width, height) != RectangleOut) )
	{
		/* Forces entire thumb to be painted. */
		w->slider.startLoc = -(w->slider.length + 1);
		PaintThumb( w ); 
	        PaintValue( w );
	}

	XSetFillStyle(XtDisplay(w), w->slider.gc, FillSolid);
	if (w->slider.orientation == XtorientHorizontal)
	{
/*
		printf("x = %ld\n", w->slider.x);
		printf("y = %ld\n", w->slider.y);
		printf("w = %ld\n", w->core.width);
		printf("h = %ld\n", w->core.height);
		printf("t = %ld\n", w->slider.thickness);
*/
                if(w->slider.label[0] != '\0') {
		XSetLineAttributes(XtDisplay(w), w->slider.gc,
				   2, LineSolid, CapNotLast, JoinMiter);
		XDrawRectangle(XtDisplay(w), XtWindow(w), w->slider.gc,
			w->slider.x-1, w->slider.y-1,
			w->slider.length+w->slider.min_thumb+1,
			w->slider.thickness+1);
		XSetLineAttributes(XtDisplay(w), w->slider.gc,
				   1, LineSolid, CapNotLast, JoinMiter);

		if(w->slider.int_slider == TRUE)
		  sprintf(labelstr, "%d", (int) w->slider.min);
		else 
		  sprintf(labelstr, "%4.2f", w->slider.min);
		sprintf(label, " ]  ");
		if(strlen(labelstr)<11)
			for(i=0; i<(7-strlen(labelstr)); i++)
				strcat(label, " ");
		strcat(label, labelstr);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			w->slider.x-90, hlpos,
			label, 11);

		if(w->slider.int_slider == TRUE)
		  sprintf(labelstr, "%d", (int) w->slider.max);
		else 
		  sprintf(labelstr, "%4.2f", w->slider.max);
		sprintf(label, "");
		if(strlen(labelstr)<7)
			for(i=0; i<(7-strlen(labelstr)); i++)
				strcat(label, " ");
		strcat(label, labelstr);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			w->slider.x+w->slider.length+10, hlpos,
			label, 7);

		sprintf(label, "%s  [ ", w->slider.label);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			10, hlpos,
			label, strlen(label));
		}
	}
	else
	{
                if(w->slider.label[0] != '\0') {
		XSetLineAttributes(XtDisplay(w), w->slider.gc,
				   2, LineSolid, CapNotLast, JoinMiter);
		XDrawRectangle(XtDisplay(w), XtWindow(w), w->slider.gc,
			w->slider.x-1, w->slider.y-1-w->slider.length,
			w->slider.thickness+1,
			w->slider.length+w->slider.min_thumb+1);
		XSetLineAttributes(XtDisplay(w), w->slider.gc,
				   1, LineSolid, CapNotLast, JoinMiter);

		if(w->slider.int_slider == TRUE)
		  sprintf(labelstr, "%d", (int) w->slider.min);
		else 
		  sprintf(labelstr, "%4.2f", w->slider.min);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			w->slider.x-2*font_width,
			w->slider.y+2*font_height,
			labelstr, strlen(labelstr));

		if(w->slider.int_slider == TRUE)
		  sprintf(labelstr, "%d", (int) w->slider.max);
		else 
		  sprintf(labelstr, "%4.2f", w->slider.max);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			w->slider.x-2*font_width,
			w->slider.y-w->slider.length-font_height,
			labelstr, strlen(labelstr));

		sprintf(label, "%s", w->slider.label);
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			vlpos+2*font_width, 2*font_height,
			label, strlen(label));

		sprintf(label, "[");
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			vlpos, 4*font_height,
			label, strlen(label));

		sprintf(label, "]");
		XDrawString(XtDisplay(w), XtWindow(w), w->slider.gc,
			vlpos+13*font_width, 4*font_height,
			label, strlen(label));
		}
	}
/*
	XSetFillStyle(XtDisplay(w), w->slider.gc, FillOpaqueStippled);
*/
	PaintThumb( w ); 
        PaintValue( w );
}


/* ARGSUSED */
static void StartSlide( gw, event, params, num_params )
Widget gw;
XEvent *event;
String *params;		/* direction: Back|Forward|Smooth */
Cardinal *num_params;	/* we only support 1 */
{
	SliderWidget w = (SliderWidget) gw;
	Cursor cursor;
	char direction;

	if (w->slider.direction != 0) return; /* if we're already scrolling */
	if (*num_params > 0)	direction = *params[0];
	else			direction = 'C';

	w->slider.direction = direction;
	if (w->slider.direction == 'C' && w->slider.notify_mid == FALSE)
	  return;

	switch( direction )
	{
		case 'B':
		case 'b':
			cursor = (w->slider.orientation == XtorientVertical)
				? w->slider.downCursor
				: w->slider.leftCursor;
			break;

		case 'F':
		case 'f':
			cursor = (w->slider.orientation == XtorientVertical)
				? w->slider.upCursor
				: w->slider.rightCursor;
			break;

		case 'C':
		case 'c':
			cursor = (w->slider.orientation == XtorientVertical)
				? w->slider.rightCursor
				: w->slider.upCursor;
			break;

		default:	return; /* invalid invocation */
	}

	XDefineCursor(XtDisplay(w), XtWindow(w), cursor);

	XFlush(XtDisplay(w));
}


static Boolean CompareEvents( oldEvent, newEvent )
XEvent *oldEvent, *newEvent;
{
#define Check(field) if (newEvent->field != oldEvent->field) return False;

	Check(xany.display);
	Check(xany.type);
	Check(xany.window);

	switch( newEvent->type )
	{
		case MotionNotify:
			Check(xmotion.state);
			break;
		case ButtonPress:
		case ButtonRelease:
			Check(xbutton.state);
			Check(xbutton.button);
			break;
		case KeyPress:
		case KeyRelease:
			Check(xkey.state);
			Check(xkey.keycode);
			break;
		case EnterNotify:
		case LeaveNotify:
			Check(xcrossing.mode);
			Check(xcrossing.detail);
			Check(xcrossing.state);
			break;
	}
#undef Check

	return True;
}

struct EventData {
	XEvent *oldEvent;
	int count;
};

static Bool PeekNotifyEvent( dpy, event, args )
Display *dpy;
XEvent *event;
char *args;
{
	struct EventData *eventData = (struct EventData*)args;

	return ((++eventData->count == QLength(dpy)) /* since PeekIf blocks */
		|| CompareEvents(event, eventData->oldEvent));
}


static Boolean LookAhead( w, event )
Widget w;
XEvent *event;
{
	XEvent newEvent;
	struct EventData eventData;

	if (QLength(XtDisplay(w)) == 0) return False;

	eventData.count = 0;
	eventData.oldEvent = event;

	XPeekIfEvent(XtDisplay(w), &newEvent,
		PeekNotifyEvent, (char*)&eventData);

	if (CompareEvents(event, &newEvent))	return True;
	else					return False;
}

static void ExtractPosition(w, event, x, y )
SliderWidget w;
XEvent *event;
Position *x, *y;		/* RETURN */
{
	switch( event->type )
	{
		case MotionNotify:
			*x = event->xmotion.x;	 *y = event->xmotion.y;	  break;
		case ButtonPress:
		case ButtonRelease:
			*x = event->xbutton.x;   *y = event->xbutton.y;   break;
		case KeyPress:
		case KeyRelease:
			*x = event->xkey.x;      *y = event->xkey.y;	  break;
		case EnterNotify:
		case LeaveNotify:
			*x = event->xcrossing.x; *y = event->xcrossing.y; break;
		default:
			*x = 0; *y = 0;
	}
	*x -= w->slider.x;
	*y -= w->slider.y;
}

static void NotifySlide( gw, event, params, num_params   )
Widget gw;
XEvent *event;
String *params;		/* style: Proportional|FullLength */
Cardinal *num_params;	/* we only support 1 */
{
	SliderWidget w = (SliderWidget) gw;
	int call_data;
	char style;
	Position x, y;
	double start, delta;

	if (w->slider.direction == 0) return; /* if no StartSlide */
	if (w->slider.direction == 'C' && w->slider.notify_mid == FALSE)
	  return;

	if(w->slider.inc == -999) delta = 0.01;
	else delta = w->slider.inc;
	if(delta > 0.01 || delta <= 0.0) delta = 0.01;
	if(w->slider.int_slider == TRUE) delta = 1.0;

	if (LookAhead(gw, event)) return;

	if (*num_params > 0)	style = *params[0];
	else			style = 'P';

	switch( style )
	{
		case 'P':    /* Proportional */
		case 'p':
			break;

		case 'F':    /* FullLength */
		case 'f':	
			break;
	}

	

	switch( w->slider.direction )
	{
		case 'B':
		case 'b':
			delta = -delta;
			/* fall through */
		case 'F':
		case 'f':
			if(w->slider.int_slider == TRUE) {
			  start = w->slider.value + delta;
			  if(start < w->slider.min) start = w->slider.min;
			  else if(start > w->slider.max) start = w->slider.max;
			  start = (start-w->slider.min) /
			    (w->slider.max-w->slider.min);
			}
			else {
			  if(w->slider.inc == -999)
			    start = w->slider.start+delta;
			  else 
			    start = (w->slider.value+delta-w->slider.min)
			      /(w->slider.max-w->slider.min);
			}
			w->slider.start = DoubleInRange(start, 0.0, 1.0); 
			w->slider.value = w->slider.start
			  *(w->slider.max-w->slider.min)+w->slider.min;
			w->slider.int_value = (int) w->slider.value;
			PaintThumb(w);
			PaintValue(w);
			break;

		case 'C':
		case 'c':
			/* NotifyThumb has already called the thumbProc(s) */
			break;
	}
}

/* ARGSUSED */
static void EndSlide(gw, event, params, num_params )
Widget gw;
XEvent *event;		/* unused */
String *params;		/* unused */
Cardinal *num_params;	/* unused */
{
	SliderWidget w = (SliderWidget) gw;

	XDefineCursor(XtDisplay(w), XtWindow(w), w->slider.inactiveCursor);
	XFlush(XtDisplay(w));

	w->slider.direction = 0;
}


/* ARGSUSED */
static void MoveThumb( gw, event, params, num_params )
Widget gw;
XEvent *event;
String *params;		/* unused */
Cardinal *num_params;	/* unused */
{
	SliderWidget w = (SliderWidget) gw;
	Position x, y;

	if (w->slider.direction == 0) return; /* if no StartSlide */
	if (w->slider.direction == 'C' && w->slider.notify_mid == FALSE)
	  return;

	if (LookAhead(gw, event)) return;

	ExtractPosition(w, event, &x, &y );
/*
	if ((w->slider.orientation == XtorientHorizontal
		&& y>w->slider.thickness)
	|| (w->slider.orientation == XtorientVertical
		&& x>w->slider.thickness))
			return;
*/

	w->slider.start = PercentLoc(w, x, y);
	w->slider.value = w->slider.start
		*(w->slider.max-w->slider.min)+w->slider.min;
	w->slider.int_value = (int) w->slider.value;
	PaintThumb(w);
	PaintValue(w);
	XFlush(XtDisplay(w));	/* re-draw it before Notifying */
}

/* ARGSUSED */
static void NotifyThumb( gw, event, params, num_params )
Widget gw;
XEvent *event;
String *params;		/* style: Motion|Up */
Cardinal *num_params;	/* we only support 1 */
{
	register SliderWidget w = (SliderWidget) gw;
	char style;
	static double ostart;
	static double ovalue;
	Position x, y;

	if (w->slider.direction == 0) return; /* if no StartSlide */
	if (w->slider.direction == 'C' && w->slider.notify_mid == FALSE)
	  return;

	if (LookAhead(gw, event)) return;

	if (*num_params > 0)	style = *params[0];
	else			style = 'U';

	switch( style )
	{
		case 'M':    /* Motion Event */
		case 'm':
			if (w->slider.notify_cont == FALSE) break;
			/* else fall through */

		case 'U':    /* Up Event */
		case 'u':	
			if(w->slider.int_slider) {
			  XtCallCallbacks( gw, XtNvalueProc,
					  &w->slider.int_value);
			}
			else
			  XtCallCallbacks( gw, XtNvalueProc, &w->slider.value);
			break;
	}

/*
	ExtractPosition(w, event, &x, &y );

	if ((w->slider.orientation == XtorientHorizontal
		&& y>w->slider.thickness)
	|| (w->slider.orientation == XtorientVertical
		&& x>w->slider.thickness))
	{
		w->slider.start = ostart;
		w->slider.value = ovalue;
		PaintThumb(w);
		PaintValue(w);
	}
	else
	{
		ostart = w->slider.start;
		ovalue = w->slider.value;
	}
*/
}

/************************************************************
 *
 *  Public routines. 
 *
 ************************************************************/

/* Set the scroll bar to the given location. */

extern void 
XawSliderSetValue( gw, value)
Widget gw;
double value;
{
	SliderWidget w = (SliderWidget)gw;
	double	start;


	if (w->slider.direction == 'c') return; /* if still thumbing */

	if(value > w->slider.max) value = w->slider.max;
	else if(value < w->slider.min) value = w->slider.min;
	start = (value-w->slider.min)/(w->slider.max-w->slider.min);
	w->slider.start = (start > 1.0) ? 1.0 : (start >= 0.0) ? start :
		w->slider.start;
	w->slider.value = w->slider.start
		*(w->slider.max-w->slider.min)+w->slider.min;
	w->slider.int_value = (int) w->slider.value;

	PaintThumb( w );
	PaintValue( w );
}

extern void 
XawIntSliderSetValue( gw, ivalue)
Widget gw;
int ivalue;
{
	SliderWidget w = (SliderWidget)gw;
	double	start;
	double value = (double) ivalue;

	if (w->slider.direction == 'c') return; /* if still thumbing */

	if(value > w->slider.max) value = w->slider.max;
	else if(value < w->slider.min) value = w->slider.min;
	start = (value-w->slider.min)/(w->slider.max-w->slider.min);
	w->slider.start = (start > 1.0) ? 1.0 : (start >= 0.0) ? start :
		w->slider.start;
	w->slider.value = w->slider.start
		*(w->slider.max-w->slider.min)+w->slider.min;
	w->slider.int_value = (int) w->slider.value;

	PaintThumb( w );
	PaintValue( w );
}

static void ZeroSlide( gw, event, params, num_params )
Widget gw;
XEvent *event;
String *params;
Cardinal *num_params;
{
	register SliderWidget w = (SliderWidget) gw;
	static char	buff[128] = {"\0"};
	char	c;
	KeySym	key;
	double	value = 0.0;
	static int	i=0;

	if (w->slider.notify_mid == FALSE) return;
	if(!i) bzero(buff, 128);
	XLookupString((XKeyEvent*) event, &c, 1, &key, 0);
/*
	printf("type = %d %c %d\n", i, c, c);
*/
	if(isdigit(c) || c == '.' || c == '-') {
		buff[i] = c;
/*
		printf("in = %d %c %s\n", i, buff[i], buff);
*/
		sscanf(buff, "%lf", &value);
		XawSliderSetValue( w, value);
		i++;
	}
	else if(i && c == '\b') {
		i--;
		buff[i] = '\0';
		sscanf(buff, "%lf", &value);
		XawSliderSetValue( w, value);
	}
	else if(c == '\r') {
/*
		printf("out = %d %s\n", i, buff);
*/
		if(i) sscanf(buff, "%lf", &value);
		i = 0;
		XawSliderSetValue( w, value);
		XtCallCallbacks( (Widget) w, XtNvalueProc, &w->slider.value);
	}
}


XkwReconfigureSlider(widget, label, min, max, val)
Widget widget;
char *label;
double min, max, val;
{
  SliderWidget w = (SliderWidget) widget;
  double start;
  XClearArea(XtDisplay(w), XtWindow(w),
	     0, 0, w->core.width, w->core.height, FALSE);
  strcpy(w->slider.label, label);
  w->slider.max = max;
  w->slider.min = min;
  start = (val-w->slider.min)/(w->slider.max-w->slider.min);
  w->slider.start = (start > 1.0) ? 1.0 : (start >= 0.0) ? start :
    w->slider.start;
  w->slider.value = w->slider.start
    *(w->slider.max-w->slider.min)+w->slider.min;
  w->slider.int_value = (int) w->slider.value;

  SetDimensions( (SliderWidget)w );
  Redisplay( w, (XEvent*)NULL, (Region)NULL );
}


XkwReconfigureIntSlider(widget, label, imin, imax, ival)
Widget widget;
char *label;
int imin, imax, ival;
{
  SliderWidget w = (SliderWidget) widget;
  double start;
  double min, max, val;

  min = (double) imin;
  max = (double) imax;
  val = (double) ival;

  XClearArea(XtDisplay(w), XtWindow(w),
	     0, 0, w->core.width, w->core.height, FALSE);
  strcpy(w->slider.label, label);
  w->slider.max = max;
  w->slider.min = min;
  start = (val-w->slider.min)/(w->slider.max-w->slider.min);
  w->slider.start = (start > 1.0) ? 1.0 : (start >= 0.0) ? start :
    w->slider.start;
  w->slider.value = w->slider.start
    *(w->slider.max-w->slider.min)+w->slider.min;
  w->slider.int_value = (int) w->slider.value;

  SetDimensions( (SliderWidget)w );
  Redisplay( w, (XEvent*)NULL, (Region)NULL );
}


