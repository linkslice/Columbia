// Columbia — a tiny stand-in for the parts of the Haiku Interface Kit that
// the custom views use, backed by Cairo. Lets tests/preview render the real
// DSKYView and TrajectoryView code to PNG on a non-Haiku host.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#ifndef HAIKU_SHIM_H
#define HAIKU_SHIM_H

#include <cairo.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

typedef int32_t int32;
typedef uint32_t uint32;
typedef uint8_t uint8;
typedef unsigned char uchar;
typedef int64_t bigtime_t;
typedef int32 status_t;
const status_t B_OK = 0;

struct rgb_color {
	uint8 red, green, blue, alpha;
	bool operator==(const rgb_color& o) const
	{ return red == o.red && green == o.green && blue == o.blue
		&& alpha == o.alpha; }
};
const rgb_color B_TRANSPARENT_COLOR = {0x77, 0x74, 0x77, 0};

struct BPoint {
	float x = 0, y = 0;
	BPoint() = default;
	BPoint(float x_, float y_) : x(x_), y(y_) {}
	BPoint operator+(const BPoint& o) const { return {x + o.x, y + o.y}; }
	BPoint operator-(const BPoint& o) const { return {x - o.x, y - o.y}; }
};

struct BSize {
	float width, height;
	BSize(float w = 0, float h = 0) : width(w), height(h) {}
};

struct BRect {
	float left = 0, top = 0, right = -1, bottom = -1;
	BRect() = default;
	BRect(float l, float t, float r, float b)
		: left(l), top(t), right(r), bottom(b) {}
	BRect(BPoint a, BPoint b) : left(a.x), top(a.y), right(b.x), bottom(b.y) {}
	float Width() const { return right - left; }
	float Height() const { return bottom - top; }
	BRect InsetByCopy(float dx, float dy) const
	{ return BRect(left + dx, top + dy, right - dx, bottom - dy); }
	BRect OffsetByCopy(float dx, float dy) const
	{ return BRect(left + dx, top + dy, right + dx, bottom + dy); }
	bool Contains(BPoint p) const
	{ return p.x >= left && p.x <= right && p.y >= top && p.y <= bottom; }
	BPoint LeftTop() const { return BPoint(left, top); }
};

struct pattern { uint8 data[8]; };
const pattern B_SOLID_HIGH = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};

struct font_height { float ascent, descent, leading; };

class BFont {
public:
	BFont(bool bold = false, bool fixed = false)
		: fSize(12), fBold(bold), fFixed(fixed) {}
	BFont(const BFont* other)
		: fSize(other->fSize), fBold(other->fBold), fFixed(other->fFixed) {}
	void SetSize(float size) { fSize = size; }
	float Size() const { return fSize; }
	bool Bold() const { return fBold; }
	bool Fixed() const { return fFixed; }
	void GetHeight(font_height* h) const
	{ h->ascent = fSize * 0.76f; h->descent = fSize * 0.24f;
		h->leading = fSize * 0.1f; }
private:
	float fSize;
	bool fBold;
	bool fFixed;
};
extern const BFont* be_plain_font;
extern const BFont* be_bold_font;
extern const BFont* be_fixed_font;

enum color_which {
	B_PANEL_BACKGROUND_COLOR, B_PANEL_TEXT_COLOR, B_LIST_BACKGROUND_COLOR,
	B_LIST_SELECTED_BACKGROUND_COLOR, B_LIST_ITEM_TEXT_COLOR,
	B_LIST_SELECTED_ITEM_TEXT_COLOR
};
rgb_color ui_color(color_which which);
rgb_color tint_color(rgb_color color, float tint);
const float B_LIGHTEN_1_TINT = 0.590f;
const float B_LIGHTEN_2_TINT = 0.385f;
const float B_DARKEN_2_TINT = 1.288f;

enum drawing_mode { B_OP_COPY, B_OP_OVER, B_OP_ALPHA };
enum { B_RGB32 = 1 };
typedef int color_space;

const uint32 B_WILL_DRAW = 1, B_NAVIGABLE = 2, B_FRAME_EVENTS = 4,
	B_FOLLOW_NONE = 0, B_POINTER_EVENTS = 1, B_LOCK_WINDOW_FOCUS = 2;
const uint32 B_MOUSE_WHEEL_CHANGED = 'Bmsw';
const char B_BACKSPACE = 0x08, B_ENTER = 0x0a;

bigtime_t system_time();

class BRegion {
public:
	BRegion() = default;
	explicit BRegion(BRect r) : fRect(r), fSet(true) {}
	void Set(BRect r) { fRect = r; fSet = true; }
	BRect Frame() const { return fRect; }
	bool IsSet() const { return fSet; }
private:
	BRect fRect;
	bool fSet = false;
};

class BMessage {
public:
	uint32 what = 0;
	status_t FindFloat(const char*, float*) const { return -1; }
	status_t FindInt32(const char*, int32*) const { return -1; }
};

class BWindow {
public:
	BMessage* CurrentMessage() const { return nullptr; }
};

class BView {
public:
	BView(const char* name, uint32 flags);
	BView(BRect frame, const char* name, uint32 resize, uint32 flags);
	virtual ~BView() {}

	virtual void Draw(BRect) {}
	virtual void AttachedToWindow() {}
	virtual void FrameResized(float, float) {}
	virtual void MouseDown(BPoint) {}
	virtual void MouseUp(BPoint) {}
	virtual void MouseMoved(BPoint, uint32, const BMessage*) {}
	virtual void KeyDown(const char*, int32) {}
	virtual void KeyUp(const char*, int32) {}
	virtual void MessageReceived(BMessage*) {}
	virtual BSize MinSize() { return BSize(); }
	virtual BSize MaxSize() { return BSize(); }
	virtual BSize PreferredSize() { return BSize(); }

	BRect Bounds() const { return fBounds; }
	void ResizeTo(float w, float h) { fBounds = BRect(0, 0, w, h); }
	BWindow* Window() const { return nullptr; }
	void Invalidate() {}
	void Invalidate(BRect) {}
	void MakeFocus(bool = true) {}
	void SetMouseEventMask(uint32, uint32 = 0) {}
	void GetMouse(BPoint* p, uint32* b, bool = true) { *p = BPoint(); *b = 0; }
	void Sync() {}
	void SetViewColor(rgb_color) {}
	void SetLowColor(rgb_color) {}

	void SetHighColor(rgb_color c) { fHigh = c; }
	void SetHighColor(uchar r, uchar g, uchar b, uchar a = 255)
	{ fHigh = rgb_color{r, g, b, a}; }
	void SetDrawingMode(drawing_mode m) { fMode = m; }
	void SetPenSize(float s) { fPen = s; }
	void SetFont(const BFont* font) { fFont = *font; }
	float StringWidth(const char* text) const;

	void FillRect(BRect r);
	void StrokeRect(BRect r);
	void FillRoundRect(BRect r, float rx, float ry);
	void StrokeRoundRect(BRect r, float rx, float ry);
	void FillEllipse(BPoint c, float rx, float ry);
	void StrokeEllipse(BPoint c, float rx, float ry,
		pattern p = B_SOLID_HIGH);
	void FillPolygon(const BPoint* points, int32 count);
	void StrokeLine(BPoint a, BPoint b);
	void DrawString(const char* text, BPoint where);
	void BeginLineArray(int32) {}
	void AddLine(BPoint a, BPoint b, rgb_color color);
	void EndLineArray() {}
	void DrawBitmap(const class BBitmap* bitmap, BRect, BRect);
	void DrawBitmap(const class BBitmap* bitmap, BPoint);
	void ConstrainClippingRegion(BRegion* region);
	void PushState();
	void PopState();
	void SetOrigin(BPoint p);
	void SetScale(float scale);

	// Shim only: the Cairo context this view draws into.
	void SetContext(cairo_t* cr) { fCairo = cr; }

private:
	void _Color();
	void _RoundRect(BRect r, float rad);

	BRect fBounds;
	cairo_t* fCairo = nullptr;
	rgb_color fHigh = {0, 0, 0, 255};
	drawing_mode fMode = B_OP_COPY;
	float fPen = 1;
	BFont fFont;
};

class BBitmap {
public:
	BBitmap(BRect bounds, color_space, bool);
	~BBitmap();
	void AddChild(BView* view);
	bool Lock() { return true; }
	void Unlock() {}
	BRect Bounds() const { return fBounds; }
	cairo_surface_t* Surface() const { return fSurface; }
private:
	BRect fBounds;
	cairo_surface_t* fSurface;
	cairo_t* fCairo;
};

#endif
