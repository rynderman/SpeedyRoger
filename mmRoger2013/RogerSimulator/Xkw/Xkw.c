/*
**
**      widgets.c:	
**      author:		Kamal Souccar
**      date:		Mar 26, 1991
**
*/

#include <stdio.h>
#include <math.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/cursorfont.h>
#include <X11/Xatom.h>
#include <X11/Shell.h>
#include <X11/Xaw/Paned.h>
#include <X11/Xaw/Simple.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Dialog.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/List.h>
#include <X11/Xaw/MenuButton.h>
#include <X11/Xaw/SimpleMenu.h>
#include <X11/Xaw/Sme.h>
#include <X11/Xaw/SmeBSB.h>
#include "Canvas.h"
#include "Slider.h"

#define ScrollX (LASTEvent+1)
#define ScrollY (LASTEvent+2)

#define P	printf("%ld\n", __LINE__);

Widget	XkwMakePane();
Widget	XkwMakeForm();
Widget	XkwMakeLabel();
Widget	XkwMakeCommand();
Widget	XkwMakeCommandB();
Widget	XkwMakeCommandR();
Widget	XkwMakeToggle();
Widget	XkwMakeDialog();
Widget	XkwMakeList();
Widget	XkwMakeSlider();
Widget	XkwMakeIntSlider();
Widget	XkwMakeSafeSlider();
Widget	XkwMakeCanvas();
Widget	XkwMakePopup();
double	XkwGetSliderValue();
void	XkwSetSliderValue();
void	XkwSetSliderSafe();
int	XkwGetIntSliderValue();
void	XkwSetIntSliderValue();
void	XkwSetWidgetLabel();
void	XkwSetWidgetColor();
void	XkwGetWidgetColor();
void    XkwSetWidgetFont();
void    XkwSetWidgetBorderColor();
void	XkwWidgetHighlight();
void	XkwWidgetUnhighlight();
Widget  XkwMakeMenuButton();
Widget  XkwMakeSimpleMenu();
Widget  XkwMakeSmeBSB();
Widget  XkwMakeScrollCanvas();
int     XkwGetSliderOrient();
unsigned long XkwParseColor();
Widget  XkwMakeShell();
void    XkwGetWidgetDim();

Widget
XkwMakePane(parent)
Widget	parent;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNx, 0);					argn++;
	XtSetArg(args[argn], XtNy, 0);					argn++;
	widget = XtCreateManagedWidget("paned", panedWidgetClass,
	  	parent, args, argn);

	return(widget);
}


Widget
XkwMakeForm(parent)
Widget	parent;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	widget = XtCreateManagedWidget("form", formWidgetClass,
 		parent, args, argn);
	return(widget);
}

Widget
XkwMakeLabel(parent, vert, horiz, label, width, height)
Widget	parent, vert, horiz;
char	label[256];
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	XtSetArg(args[argn], XtNlabel, label);				argn++;
	widget = XtCreateManagedWidget("label", labelWidgetClass,
		parent, args, argn);

	return(widget);
}

Widget
XkwMakeCommand(parent, vert, horiz, callbackproc, label, width, height)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	label[128];
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	widget = XtCreateManagedWidget(label, commandWidgetClass,
	  	parent, args, argn);
	XtAddCallback(widget, XtNcallback, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeCommandBR(parent, vert, horiz, vd, hd, bg, fg, pixmap, 
		callbackproc, label)
Widget	parent, vert, horiz;
int vd, hd, bg, fg;
Pixmap pixmap;
XtCallbackProc	callbackproc;
char *label;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNvertDistance, vd);  argn++;
	XtSetArg(args[argn], XtNhorizDistance, hd);  argn++;
/*
	XtSetArg(args[argn], XtNshapeStyle, XawShapeEllipse);  argn++;
*/
	XtSetArg(args[argn], XtNbitmap, pixmap);			argn++;
	XtSetArg(args[argn], XtNbackground, (Pixel) bg);                argn++;
	XtSetArg(args[argn], XtNforeground, (Pixel) fg);                argn++;
	widget = XtCreateManagedWidget(label, commandWidgetClass,
	  	parent, args, argn);
	XtAddCallback(widget, XtNcallback, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeCommandR(parent, vert, horiz, vd, hd, 
		callbackproc, label, width, height)
Widget	parent, vert, horiz;
int hd, vd;
XtCallbackProc	callbackproc;
char	label[128];
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
/*
#define XawShapeRectangle XmuShapeRectangle
#define XawShapeOval XmuShapeOval
#define XawShapeEllipse XmuShapeEllipse
#define XawShapeRoundedRectangle XmuShapeRoundedRectangle
*/
	XtSetArg(args[argn], XtNshapeStyle, XawShapeEllipse);  argn++;
	XtSetArg(args[argn], XtNvertDistance, vd);  argn++;
	XtSetArg(args[argn], XtNhorizDistance, hd);  argn++;
	widget = XtCreateManagedWidget(label, commandWidgetClass,
	  	parent, args, argn);
	XtAddCallback(widget, XtNcallback, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeDialog(parent, vert, horiz, label, value)
Widget	parent, vert, horiz;
char	*label;
char	*value;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNvalue, value);				argn++;
	XtSetArg(args[argn], XtNlabel, label);				argn++;
	widget = XtCreateManagedWidget(label, dialogWidgetClass,
	  	parent, args, argn);

	return(widget);
}

Widget
XkwMakeToggle(parent, vert, horiz, callbackproc, label, width, height)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	label[128];
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	widget = XtCreateManagedWidget(label, toggleWidgetClass,
	  	parent, args, argn);
	XtAddCallback(widget, XtNcallback, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeList(parent, vert, horiz, callbackproc, label, items, width, height)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	label[128];
String	*items;
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNlist, items);				argn++;
/*
	XtSetArg(args[argn], XtNverticalList, True);			argn++;
*/
	XtSetArg(args[argn], XtNdefaultColumns, 5);			argn++;
	XtSetArg(args[argn], XtNforceColumns, True);			argn++;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	widget = XtCreateManagedWidget(label, listWidgetClass,
	  	parent, args, argn);
	XtAddCallback(widget, XtNcallback, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeSlider(parent, vert, horiz, callbackproc,
		label, width, height, orientation,
		value, min, max, bool, attach)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	label[128];
int	width, height;
XtOrientation orientation;
double	value, min, max;
Boolean	bool;
int	attach;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;
	XtArgVal 	* l_min, * l_max, * l_value;

	argn = 0;
	if (sizeof(double) > sizeof(XtArgVal)) {
	  XtSetArg(args[argn], XtNmin,            &min);		argn++;
	  XtSetArg(args[argn], XtNmax,            &max);		argn++;
	  XtSetArg(args[argn], XtNvalue,          &value);		argn++;
	}
	else {
	  l_min = (XtArgVal *) &min;
	  l_max = (XtArgVal *) &max;
	  l_value = (XtArgVal *) &value;
	  XtSetArg(args[argn], XtNmin,            *l_min);		argn++;
	  XtSetArg(args[argn], XtNmax,            *l_max);		argn++;
	  XtSetArg(args[argn], XtNvalue,          *l_value);		argn++;
	}
	XtSetArg(args[argn], XtNlabel,		label);			argn++;
	XtSetArg(args[argn], XtNlength,		200);			argn++;
	XtSetArg(args[argn], XtNwidth,		width);			argn++;
	XtSetArg(args[argn], XtNheight,		height);		argn++;
	XtSetArg(args[argn], XtNfromHoriz,	horiz);			argn++;
	XtSetArg(args[argn], XtNfromVert,	vert);			argn++;
	XtSetArg(args[argn], XtNnotifyCont,	bool);			argn++;
	XtSetArg(args[argn], XtNorientation,	orientation);		argn++;
	XtSetArg(args[argn], XtNattachThumb,	attach);		argn++;
	XtSetArg(args[argn], XtNnotifyMid,	TRUE);			argn++;
	widget = XtCreateManagedWidget(label, sliderWidgetClass,
		parent, args, argn);
	XtAddCallback(widget, XtNvalueProc, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeIntSlider(parent, vert, horiz, callbackproc,
		label, width, height, orientation,
		ivalue, imin, imax, bool, attach)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	label[128];
int	width, height;
XtOrientation orientation;
int	ivalue, imin, imax;
Boolean	bool;
int	attach;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;
	double	value, min, max;
	XtArgVal 	* l_min, * l_max, * l_value;

	value = (double) ivalue;
	min = (double) imin;
	max = (double) imax;

	argn = 0;
	if (sizeof(double) > sizeof(XtArgVal)) {
	  XtSetArg(args[argn], XtNmin,            &min);		argn++;
	  XtSetArg(args[argn], XtNmax,            &max);		argn++;
	  XtSetArg(args[argn], XtNvalue,          &value);		argn++;
	}
	else {
	  l_min = (XtArgVal *) &min;
	  l_max = (XtArgVal *) &max;
	  l_value = (XtArgVal *) &value;
	  XtSetArg(args[argn], XtNmin,            *l_min);		argn++;
	  XtSetArg(args[argn], XtNmax,            *l_max);		argn++;
	  XtSetArg(args[argn], XtNvalue,          *l_value);		argn++;
	}
	XtSetArg(args[argn], XtNlabel,		label);			argn++;
	XtSetArg(args[argn], XtNlength,		200);			argn++;
	XtSetArg(args[argn], XtNwidth,		width);			argn++;
	XtSetArg(args[argn], XtNheight,		height);		argn++;
	XtSetArg(args[argn], XtNfromHoriz,	horiz);			argn++;
	XtSetArg(args[argn], XtNfromVert,	vert);			argn++;
	XtSetArg(args[argn], XtNnotifyCont,	bool);			argn++;
	XtSetArg(args[argn], XtNorientation,	orientation);		argn++;
	XtSetArg(args[argn], XtNattachThumb,	attach);		argn++;
	XtSetArg(args[argn], XtNintSlider,	TRUE);     		argn++;
	XtSetArg(args[argn], XtNnotifyMid,	TRUE);			argn++;
	widget = XtCreateManagedWidget(label, sliderWidgetClass,
		parent, args, argn);
	XtAddCallback(widget, XtNvalueProc, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeSafeSlider(parent, vert, horiz, callbackproc,
		label, width, height, orientation,
		value, min, max, inc, attach)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	label[128];
int	width, height;
XtOrientation orientation;
double	value, min, max, inc;
int	attach;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;
	XtArgVal 	* l_min, * l_max, * l_value, * l_inc;

	argn = 0;
	if (sizeof(double) > sizeof(XtArgVal)) {
	  XtSetArg(args[argn], XtNmin,            &min);		argn++;
	  XtSetArg(args[argn], XtNmax,            &max);		argn++;
	  XtSetArg(args[argn], XtNvalue,          &value);		argn++;
	  XtSetArg(args[argn], XtNinc,            &inc);		argn++;
	}
	else {
	  l_min = (XtArgVal *) &min;
	  l_max = (XtArgVal *) &max;
	  l_value = (XtArgVal *) &value;
	  l_inc = (XtArgVal *) &inc;
	  XtSetArg(args[argn], XtNmin,            *l_min);		argn++;
	  XtSetArg(args[argn], XtNmax,            *l_max);		argn++;
	  XtSetArg(args[argn], XtNvalue,          *l_value);		argn++;
	  XtSetArg(args[argn], XtNinc,            *l_inc);		argn++;
	}
	XtSetArg(args[argn], XtNlabel,		label);			argn++;
	XtSetArg(args[argn], XtNlength,		200);			argn++;
	XtSetArg(args[argn], XtNwidth,		width);			argn++;
	XtSetArg(args[argn], XtNheight,		height);		argn++;
	XtSetArg(args[argn], XtNfromHoriz,	horiz);			argn++;
	XtSetArg(args[argn], XtNfromVert,	vert);			argn++;
	XtSetArg(args[argn], XtNnotifyMid,	FALSE);			argn++;
	XtSetArg(args[argn], XtNnotifyCont,	FALSE);			argn++;
	XtSetArg(args[argn], XtNorientation,	orientation);		argn++;
	XtSetArg(args[argn], XtNattachThumb,	attach);		argn++;
	widget = XtCreateManagedWidget(label, sliderWidgetClass,
		parent, args, argn);
	XtAddCallback(widget, XtNvalueProc, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeCanvas(parent, vert, horiz, callbackproc, width, height, label)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	widget = XtCreateManagedWidget("canvas", canvasWidgetClass,
	  	parent, args, argn);
	XtAddCallback(widget, XtNcallback, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeScrollCanvas(parent, vert, horiz, callbackproc,
        width, height, max_w, max_h)
Widget  parent, vert, horiz;
XtCallbackProc  callbackproc;
int     width, height, max_w, max_h;
{
        Arg             args[15];
        Cardinal        argn;
        Widget          canvas, form, x, y;

        if(width < max_w && height < max_w) {
        argn = 0;
        XtSetArg(args[argn], XtNfromVert, vert);                        argn++;
        XtSetArg(args[argn], XtNfromHoriz, horiz);                      argn++;
        form = XtCreateManagedWidget("form", formWidgetClass,
                parent, args, argn);
        }
        else {
        argn = 0;
        XtSetArg(args[argn], XtNfromVert, vert);                        argn++;
        XtSetArg(args[argn], XtNfromHoriz, horiz);                      argn++;
        form = XtCreateManagedWidget("pane", panedWidgetClass,
                parent, args, argn);
        }

        argn = 0;
        XtSetArg(args[argn], XtNfromVert, NULL);                        argn++;
        XtSetArg(args[argn], XtNfromHoriz, NULL);                       argn++;
        XtSetArg(args[argn], XtNwidth, width);                          argn++;
        XtSetArg(args[argn], XtNheight, height);                        argn++;
        canvas = XtCreateManagedWidget("canvas", canvasWidgetClass,
                form, args, argn);
        XtAddCallback(canvas, XtNcallback, callbackproc, NULL);

        if(width < max_w)
        x = XkwMakeSlider(form, canvas, NULL, callbackproc, "\0", width, 0,
        XtorientHorizontal, 0.0, 0.0, (double) width, FALSE, Self);
        if(height < max_h)
        y = XkwMakeSlider(form, NULL, canvas, callbackproc, "\0", 0, height,
        XtorientVertical, (double) height, 0.0, (double) height, FALSE, Self);

        return(canvas);
}

Widget
XkwMakePopup(parent, x, y, label)
Widget	parent;
int	x, y;
char *label;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;
	Position	xx, yy;
	Dimension	width, height;

	if(x < 0  && y < 0)
	{	
	argn = 0;
	XtSetArg(args[argn], XtNwidth, &width);				argn++;
	XtSetArg(args[argn], XtNheight, &height);			argn++;
	XtGetValues(parent, args, argn);
	XtTranslateCoords(parent,
		(Position)(width/2), (Position)(height/2), &xx, &yy);
	x = xx;
	y = yy;
	}

	argn = 0;
	XtSetArg(args[argn], XtNx, x);					argn++;
	XtSetArg(args[argn], XtNy, y);					argn++;
	widget =  XtCreatePopupShell(label, transientShellWidgetClass,
	  	parent, args, argn);

	return(widget);
}

double
XkwGetSliderValue(slider)
Widget	slider;
{
	Arg		args[15];
	Cardinal	argn;
	double		value;

	argn = 0;
	XtSetArg(args[argn], XtNvalue, &value);				argn++;
	XtGetValues(slider, args, argn);

	return(value);
}

int
XkwGetIntSliderValue(slider)
Widget	slider;
{
	Arg		args[15];
	Cardinal	argn;
	int		value;

	argn = 0;
	XtSetArg(args[argn], XtNintValue, &value);			argn++;
	XtGetValues(slider, args, argn);

	return(value);
}

void
XkwSetSliderValue(slider, value)
Widget	slider;
double	value;
{
	XawSliderSetValue(slider, value);
}

void
XkwSetIntSliderValue(slider, value)
Widget	slider;
int	value;
{
	XawIntSliderSetValue(slider, value);
}

void
XkwSetSliderSafe(widget, safe)
Widget	widget;
Boolean safe;
{
	Arg		args[15];
	Cardinal	argn;

	argn = 0;
	XtSetArg(args[argn], XtNnotifyMid, safe);			argn++;
	XtSetValues(widget, args, argn);
}

void
XkwSetWidgetLabel(widget, label)
Widget	widget;
char	*label;
{
	Arg		args[15];
	Cardinal	argn;

	argn = 0;
	XtSetArg(args[argn], XtNlabel, label);			argn++;
	XtSetValues(widget, args, argn);
}

void
XkwSetWidgetBorderColor(widget, color)
Widget	widget;
unsigned long color;
{
	Arg		args[15];
	Cardinal	argn;

	argn = 0;
	XtSetArg(args[argn], XtNborderColor, color);			argn++;
	XtSetValues(widget, args, argn);
}

void
XkwSetWidgetColor(widget, fg, bg)
Widget	widget;
unsigned long fg, bg;
{
	Arg		args[15];
	Cardinal	argn;

	argn = 0;
	XtSetArg(args[argn], XtNforeground, fg);			argn++;
	XtSetArg(args[argn], XtNbackground, bg);			argn++;
	XtSetValues(widget, args, argn);
}

void
XkwGetWidgetColor(widget, fg, bg)
Widget	widget;
unsigned long *fg, *bg;
{
	Arg		args[15];
	Cardinal	argn;

	argn = 0;
	XtSetArg(args[argn], XtNforeground, fg);			argn++;
	XtSetArg(args[argn], XtNbackground, bg);			argn++;
	XtGetValues(widget, args, argn);
}

void
XkwGetWidgetDim(widget, x, y, w, h)
Widget	widget;
int *x, *y, *w, *h;
{
	Arg		args[15];
	Cardinal	argn;
	Dimension ww, hh;

	argn = 0;
	XtSetArg(args[argn], XtNx, x);			argn++;
	XtSetArg(args[argn], XtNy, y);			argn++;
	XtSetArg(args[argn], XtNwidth, &ww);			argn++;
	XtSetArg(args[argn], XtNheight, &hh);			argn++;
	XtGetValues(widget, args, argn);
	*w = (int) ww;
	*h = (int) hh;
}

void
XkwSetWidgetFont(widget, font_struct)
Widget	widget;
XFontStruct * font_struct;
{
	Arg		args[15];
	Cardinal	argn;

	argn = 0;
	XtSetArg(args[argn], XtNfont, font_struct);			argn++;
	XtSetValues(widget, args, argn);
}

void
XkwWidgetHighlight(widget)
Widget	widget;
{
	Arg		args[15];
	Cardinal	argn;
        long            fg, bg;
        Display         *display;
        int             screen;

/*
        display = XtDisplay(widget);
        screen = DefaultScreen(display);
        fg = BlackPixel(display, screen);
        bg = WhitePixel(display, screen);
*/

	argn = 0;
	XtSetArg(args[argn], XtNforeground, (Pixel) &fg);		argn++;
	XtSetArg(args[argn], XtNbackground, (Pixel) &bg);		argn++;
	XtGetValues(widget, args, argn);

	argn = 0;
	XtSetArg(args[argn], XtNforeground, (Pixel) bg);		argn++;
	XtSetArg(args[argn], XtNbackground, (Pixel) fg);		argn++;
	XtSetValues(widget, args, argn);
}

void
XkwWidgetUnhighlight(widget)
Widget	widget;
{
	Arg		args[15];
	Cardinal	argn;
        long            fg, bg;
/*
        Display         *display;
        int             screen;

        display = XtDisplay(widget);
        screen = DefaultScreen(display);
        fg = BlackPixel(display, screen);
        bg = WhitePixel(display, screen);
*/
	argn = 0;
	XtSetArg(args[argn], XtNforeground, (Pixel) &fg);		argn++;
	XtSetArg(args[argn], XtNbackground, (Pixel) &bg);		argn++;
	XtGetValues(widget, args, argn);

	argn = 0;
	XtSetArg(args[argn], XtNforeground, (Pixel) bg);		argn++;
	XtSetArg(args[argn], XtNbackground, (Pixel) fg);		argn++;
	XtSetValues(widget, args, argn);
}


Widget
XkwMakeMenuButton(parent, vert, horiz, callbackproc, label, width, height)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	*label;
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	widget = XtCreateManagedWidget(label, menuButtonWidgetClass,
	  	parent, args, argn);

	return(widget);
}

/*
Widget
XkwMakeSimpleMenu(parent, width, height)
Widget	parent;
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	widget = XtCreatePopupShell("menu", simpleMenuWidgetClass,
	  	parent, args, argn);

	return(widget);
}
*/

Widget
XkwMakeSmeBSB(parent, callbackproc, label, width, height)
Widget	parent;
XtCallbackProc	callbackproc;
char	*label;
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget;

	argn = 0;
/*
	XtSetArg(args[argn], XtNleftMargin, 30);			argn++;
*/
	widget = XtCreateManagedWidget(label, smeBSBObjectClass,
	  	parent, args, argn);
	XtAddCallback(widget, XtNcallback, callbackproc, NULL);

	return(widget);
}

Widget
XkwMakeSimpleMenu(parent, vert, horiz, callbackproc,
		label, items, number, width, height)
Widget	parent, vert, horiz;
XtCallbackProc	callbackproc;
char	*label;
char	*items[16];
int	number;
int	width, height;
{
	Arg		args[15];
	Cardinal	argn;
	Widget		widget, command, menu;
	int		i;

	argn = 0;
	XtSetArg(args[argn], XtNfromVert, vert);			argn++;
	XtSetArg(args[argn], XtNfromHoriz, horiz);			argn++;
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
	command = XtCreateManagedWidget(label, menuButtonWidgetClass,
	  	parent, args, argn);

	argn = 0;
/*
	XtSetArg(args[argn], XtNwidth, width);				argn++;
	XtSetArg(args[argn], XtNheight, height);			argn++;
*/
	menu = XtCreatePopupShell("menu", simpleMenuWidgetClass,
	  	command, args, argn);

	for(i=0; i<number; i++) {
		char * item = items[i];
/*
		printf("%s %d\n", item, i);
*/
		argn = 0;
		XtSetArg(args[argn], XtNleftMargin, 30);		argn++;
		widget = XtCreateManagedWidget(item, smeBSBObjectClass,
	  		menu, args, argn);
		XtAddCallback(widget, XtNcallback, callbackproc, NULL);
	}

	return(command);
}

int
XkwGetSliderOrient(slider)
Widget	slider;
{
	Arg		args[15];
	Cardinal	argn;
	XtOrientation	orientation;

	argn = 0;
	XtSetArg(args[argn], XtNorientation,	&orientation);		argn++;
	XtGetValues(slider, args, argn);

	return(ScrollX+orientation);
}

unsigned long
XkwParseColor(display, colorname)
Display * display;
char *colorname;
{
  XColor defcolor, rgb_defcolor;
  Colormap colormap;

  colormap = DefaultColormap(display, DefaultScreen(display));
  if (XAllocNamedColor(display, colormap, colorname,
		       &defcolor, &rgb_defcolor)) return defcolor.pixel;
  printf("no such color \"%s\"\n",  colorname);
  return BlackPixel(display, DefaultScreen(display));
}

Widget
XkwMakeShell(display, name, x, y)
Display *display;
char * name;
int x, y;
{
  Widget widget;
  Arg args[10];
  Cardinal argn = 0;
  
  XtSetArg(args[argn], XtNx, x);  argn++;
  XtSetArg(args[argn], XtNy, y);  argn++;
  widget = XtAppCreateShell(name, name, applicationShellWidgetClass,
			    display, args, argn);
  return (widget);
}

void
XkwSetListColumns(widget, cols)
Widget	widget;
int cols;
{
  Arg		args[15];
  Cardinal	argn;

  argn = 0;
  XtSetArg(args[argn], XtNdefaultColumns, cols);			argn++;
  XtSetValues(widget, args, argn);
}

