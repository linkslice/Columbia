// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Out the window: the view from the crewed spacecraft along its window's
// line of sight -- stars, the Earth and the Moon, the other spacecraft
// (the S-IVB, the LM, the CSM) in perspective, and close to the Moon the
// surface with its craters. The CM's rendezvous window has the COAS; the
// LM commander's window has the LPD scale with the LGC's designated angle.
// Drawn with ordinary 2D views (no OpenGL), so it runs anywhere Haiku does.

#ifndef COLUMBIA_WINDOW_VIEW_H
#define COLUMBIA_WINDOW_VIEW_H

#include <View.h>

#include "sim/Math.h"

#include <string>
#include <vector>

namespace columbia {

class FlightSession;

class WindowView : public BView {
public:
	explicit WindowView(FlightSession* session);

	void Draw(BRect updateRect) override;
	void FrameResized(float width, float height) override;
	void Refresh();

	// Look a little off the window's axis (degrees), from the mouse.
	void MouseDown(BPoint where) override;
	void MouseMoved(BPoint where, uint32 transit, const BMessage* drag) override;
	void MouseUp(BPoint where) override;

private:
	struct Other {
		sim::Vec3 r;
		sim::Quat q;
		uint16_t modules = 0;
		bool lm = false;
		bool engine = false;
		std::string name;
	};
	struct Crater {
		sim::Vec3 fixed;      // Moon-fixed position (m)
		double diameter;
	};

	void _Snapshot();
	BPoint _Project(const sim::Vec3& inertial, bool* front) const;
	void _DrawStars();
	void _DrawBody(bool moon);
	void _DrawSurface();
	void _DrawGround(const sim::Vec3& center, double radius, double distance,
		bool moon);
	void _DrawOther(const Other& o);
	void _DrawOverlay();

	FlightSession* fSession;
	bool fValid = false;
	bool fLM = false;
	sim::Vec3 fEye, fFwd, fUp, fRight;
	double fFocal = 500;
	double fJD = 0;
	sim::Vec3 fMoon;
	int fMission = 0;
	double fAltitude = 0;
	double fLPDAngle = -1;        // degrees, when the LGC shows one
	double fRange = -1, fRate = 0;
	std::string fTargetName;
	std::vector<Other> fOthers;
	std::vector<sim::Vec3> fStars;
	std::vector<float> fStarSize;
	std::vector<Crater> fCraters;
	bool fNearMoon = false;
	sim::Vec3 fSiteUp;
	double fSiteRadius = 1737400;
	double fLookYaw = 0, fLookPitch = 0;
	bool fDragging = false;
	BPoint fDragFrom;
};

}  // namespace columbia

#endif
