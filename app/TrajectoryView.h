// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Plan view of the flight: the path flown so far, the predicted path
// through the planned burns, the Earth and the Moon.

#ifndef COLUMBIA_TRAJECTORY_VIEW_H
#define COLUMBIA_TRAJECTORY_VIEW_H

#include <View.h>

#include "sim/Flight.h"

class BBitmap;

namespace columbia {
class Mission;
}

enum ViewFrame {
	FRAME_ROTATING = 0,   // Earth-Moon rotating: the Moon stays put
	FRAME_INERTIAL,       // Earth-centred, non-rotating
	FRAME_MOON            // Moon-centred, non-rotating
};

class TrajectoryView : public BView {
public:
	explicit TrajectoryView(columbia::Mission* mission);
	~TrajectoryView() override;

	void Draw(BRect updateRect) override;
	void FrameResized(float width, float height) override;
	void MouseDown(BPoint where) override;
	void MouseMoved(BPoint where, uint32 transit,
		const BMessage* dragMessage) override;
	void MouseUp(BPoint where) override;
	void MessageReceived(BMessage* message) override;
	void AttachedToWindow() override;

	BSize MinSize() override;

	void SetFrameMode(ViewFrame frame);
	ViewFrame FrameMode() const { return fFrame; }
	void SetFollow(bool follow);
	bool Follow() const { return fFollow; }
	void Fit();
	void ZoomBy(float factor, BPoint about);

	// Re-renders from the mission's current data.
	void Refresh();

private:
	sim::Vec3 _ToFrame(const sim::Vec3& r, double t) const;
	BPoint _ToView(const sim::Vec3& p) const;
	sim::Vec3 _FromView(BPoint where) const;
	void _Render();
	void _DrawPath(BView* view, const std::vector<sim::Sample>& samples,
		rgb_color color, rgb_color burnColor, bool dashed);
	void _DrawBody(BView* view, const sim::Vec3& center, double radius,
		rgb_color color, const char* label);
	void _DrawMarker(BView* view, const sim::Vec3& p, rgb_color color,
		const char* label);
	void _DrawScale(BView* view);
	void _DrawLegend(BView* view);

	columbia::Mission* fMission;
	BBitmap* fBitmap;
	BView* fOffscreen;
	ViewFrame fFrame;
	bool fFollow;
	double fScale;          // pixels per metre
	sim::Vec3 fCenter;      // frame coordinates at the view centre
	bool fDragging;
	BPoint fDragStart;
	sim::Vec3 fDragCenter;
	double fNow;            // GET of the current render
	bool fFitted;           // initial fit done (needs a real size)
};

#endif
