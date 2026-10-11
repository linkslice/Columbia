// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Recording.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace flight {

using sim::Quat;
using sim::Vec3;

namespace {

const char kMagic[8] = {'C', 'O', 'L', 'R', 'E', 'C', '0', '1'};

// --- Little-endian stream ---------------------------------------------------

class Writer {
public:
	explicit Writer(FILE* f) : fFile(f) {}
	void U8(uint8_t v) { _Put(&v, 1); }
	void U16(uint16_t v) { uint8_t b[2] = {uint8_t(v), uint8_t(v >> 8)}; _Put(b, 2); }
	void U32(uint32_t v)
	{
		uint8_t b[4];
		for (int i = 0; i < 4; i++)
			b[i] = uint8_t(v >> (8 * i));
		_Put(b, 4);
	}
	void U64(uint64_t v)
	{
		uint8_t b[8];
		for (int i = 0; i < 8; i++)
			b[i] = uint8_t(v >> (8 * i));
		_Put(b, 8);
	}
	void F64(double d) { uint64_t u; std::memcpy(&u, &d, 8); U64(u); }
	void F32(float f) { uint32_t u; std::memcpy(&u, &f, 4); U32(u); }
	void Str(const std::string& s) { U32(uint32_t(s.size())); _Put(s.data(), s.size()); }
	void Bytes(const char* p, size_t n) { _Put(p, n); }
	bool ok = true;

private:
	void _Put(const void* p, size_t n)
	{
		if (n && std::fwrite(p, 1, n, fFile) != n)
			ok = false;
	}
	FILE* fFile;
};

class Reader {
public:
	explicit Reader(FILE* f) : fFile(f) {}
	uint8_t U8() { uint8_t v = 0; _Get(&v, 1); return v; }
	uint16_t U16() { uint8_t b[2] = {}; _Get(b, 2); return uint16_t(b[0] | b[1] << 8); }
	uint32_t U32()
	{
		uint8_t b[4] = {};
		_Get(b, 4);
		uint32_t v = 0;
		for (int i = 0; i < 4; i++)
			v |= uint32_t(b[i]) << (8 * i);
		return v;
	}
	uint64_t U64()
	{
		uint8_t b[8] = {};
		_Get(b, 8);
		uint64_t v = 0;
		for (int i = 0; i < 8; i++)
			v |= uint64_t(b[i]) << (8 * i);
		return v;
	}
	double F64() { uint64_t u = U64(); double d; std::memcpy(&d, &u, 8); return d; }
	float F32() { uint32_t u = U32(); float f; std::memcpy(&f, &u, 4); return f; }
	std::string Str()
	{
		uint32_t n = U32();
		if (n > (1u << 20)) {
			ok = false;
			return std::string();
		}
		std::string s(n, '\0');
		_Get(&s[0], n);
		return s;
	}
	void Bytes(char* p, size_t n) { _Get(p, n); }
	bool ok = true;

private:
	void _Get(void* p, size_t n)
	{
		if (n && std::fread(p, 1, n, fFile) != n)
			ok = false;
	}
	FILE* fFile;
};

void
WriteDisplays(Writer& w, const std::vector<RecDisplay>& d)
{
	w.U32(uint32_t(d.size()));
	for (const RecDisplay& x : d) {
		w.F64(x.get);
		w.Bytes(x.prog, 2);
		w.Bytes(x.verb, 2);
		w.Bytes(x.noun, 2);
		w.Bytes(&x.reg[0][0], 15);
		w.Bytes(x.sign, 3);
		w.U32(x.lamps);
	}
}

bool
ReadDisplays(Reader& r, std::vector<RecDisplay>* d)
{
	uint32_t n = r.U32();
	if (!r.ok || n > 10000000)
		return false;
	d->resize(n);
	for (RecDisplay& x : *d) {
		x.get = r.F64();
		r.Bytes(x.prog, 2);
		r.Bytes(x.verb, 2);
		r.Bytes(x.noun, 2);
		r.Bytes(&x.reg[0][0], 15);
		r.Bytes(x.sign, 3);
		x.lamps = r.U32();
	}
	return r.ok;
}

Quat
Slerp(Quat a, Quat b, double t)
{
	double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
	if (d < 0) {
		b = Quat(-b.w, -b.x, -b.y, -b.z);
		d = -d;
	}
	if (d > 0.9995) {
		return Quat(a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
			a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t).Normalized();
	}
	double th = std::acos(d), s = std::sin(th);
	double ka = std::sin((1 - t) * th) / s, kb = std::sin(t * th) / s;
	return Quat(a.w * ka + b.w * kb, a.x * ka + b.x * kb, a.y * ka + b.y * kb,
		a.z * ka + b.z * kb);
}

// Cubic Hermite between (p0, v0) and (p1, v1) over h seconds.
Vec3
Hermite(const Vec3& p0, const Vec3& v0, const Vec3& p1, const Vec3& v1,
	double h, double t)
{
	double t2 = t * t, t3 = t2 * t;
	double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t;
	double h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
	return p0 * h00 + v0 * (h10 * h) + p1 * h01 + v1 * (h11 * h);
}

}  // namespace


bool
RecDisplay::SameAs(const RecDisplay& o) const
{
	return std::memcmp(prog, o.prog, 2) == 0 && std::memcmp(verb, o.verb, 2) == 0
		&& std::memcmp(noun, o.noun, 2) == 0
		&& std::memcmp(reg, o.reg, sizeof(reg)) == 0
		&& std::memcmp(sign, o.sign, 3) == 0 && lamps == o.lamps;
}


bool
Recording::Save(const std::string& path) const
{
	FILE* f = std::fopen(path.c_str(), "wb");
	if (!f)
		return false;
	Writer w(f);
	w.Bytes(kMagic, 8);
	const RecHeader& h = header;
	w.U32(uint32_t(h.missionNumber));
	w.Str(h.missionName);
	w.Str(h.csmCallsign);
	w.Str(h.lmCallsign);
	w.Str(h.siteName);
	w.F64(h.siteLatitude);
	w.F64(h.siteLongitude);
	w.F64(h.siteRadius);
	w.F64(h.splashLatitude);
	w.F64(h.splashLongitude);
	w.F64(h.getZeroJD);
	w.Str(h.difficulty);
	w.Str(h.outcome);
	w.U8(h.success ? 1 : 0);
	w.U32(uint32_t(h.contingencies.size()));
	for (const std::string& c : h.contingencies)
		w.Str(c);

	w.U32(uint32_t(samples.size()));
	for (const RecSample& s : samples) {
		w.F64(s.get);
		w.U8(s.phase);
		for (const Vec3* p : {&s.moon, &s.moonV}) {
			w.F64(p->x);
			w.F64(p->y);
			w.F64(p->z);
		}
		w.F32(float(s.altitude));
		w.F32(float(s.speed));
		for (const RecVehicle& v : s.veh) {
			w.U8(v.flags);
			if (!v.Present())
				continue;
			w.U16(v.modules);
			w.F64(v.r.x);
			w.F64(v.r.y);
			w.F64(v.r.z);
			w.F32(float(v.v.x));
			w.F32(float(v.v.y));
			w.F32(float(v.v.z));
			w.F32(float(v.q.w));
			w.F32(float(v.q.x));
			w.F32(float(v.q.y));
			w.F32(float(v.q.z));
		}
	}
	WriteDisplays(w, cmc);
	WriteDisplays(w, lgc);
	w.U32(uint32_t(events.size()));
	for (const RecEvent& e : events) {
		w.F64(e.get);
		w.U8(e.kind);
		w.Str(e.text);
	}
	bool ok = w.ok;
	if (std::fclose(f) != 0)
		ok = false;
	return ok;
}


bool
Recording::Load(const std::string& path, std::string* error)
{
	auto fail = [&](const char* why) {
		if (error)
			*error = why;
		return false;
	};
	FILE* f = std::fopen(path.c_str(), "rb");
	if (!f)
		return fail("can't open the recording");
	Reader r(f);
	char magic[8] = {};
	r.Bytes(magic, 8);
	if (!r.ok || std::memcmp(magic, kMagic, 8) != 0) {
		std::fclose(f);
		return fail("not a Columbia flight recording");
	}
	RecHeader& h = header;
	h.missionNumber = int(r.U32());
	h.missionName = r.Str();
	h.csmCallsign = r.Str();
	h.lmCallsign = r.Str();
	h.siteName = r.Str();
	h.siteLatitude = r.F64();
	h.siteLongitude = r.F64();
	h.siteRadius = r.F64();
	h.splashLatitude = r.F64();
	h.splashLongitude = r.F64();
	h.getZeroJD = r.F64();
	h.difficulty = r.Str();
	h.outcome = r.Str();
	h.success = r.U8() != 0;
	uint32_t nc = r.U32();
	if (!r.ok || nc > 1000) {
		std::fclose(f);
		return fail("damaged recording header");
	}
	h.contingencies.resize(nc);
	for (std::string& c : h.contingencies)
		c = r.Str();

	uint32_t ns = r.U32();
	if (!r.ok || ns > 50000000) {
		std::fclose(f);
		return fail("damaged recording");
	}
	samples.resize(ns);
	for (RecSample& s : samples) {
		s.get = r.F64();
		s.phase = r.U8();
		for (Vec3* p : {&s.moon, &s.moonV}) {
			p->x = r.F64();
			p->y = r.F64();
			p->z = r.F64();
		}
		s.altitude = r.F32();
		s.speed = r.F32();
		for (RecVehicle& v : s.veh) {
			v.flags = r.U8();
			if (!v.Present())
				continue;
			v.modules = r.U16();
			v.r.x = r.F64();
			v.r.y = r.F64();
			v.r.z = r.F64();
			v.v.x = r.F32();
			v.v.y = r.F32();
			v.v.z = r.F32();
			v.q.w = r.F32();
			v.q.x = r.F32();
			v.q.y = r.F32();
			v.q.z = r.F32();
		}
		if (!r.ok)
			break;
	}
	bool ok = r.ok && ReadDisplays(r, &cmc) && ReadDisplays(r, &lgc);
	if (ok) {
		uint32_t ne = r.U32();
		if (!r.ok || ne > 10000000)
			ok = false;
		else {
			events.resize(ne);
			for (RecEvent& e : events) {
				e.get = r.F64();
				e.kind = r.U8();
				e.text = r.Str();
			}
			ok = r.ok;
		}
	}
	std::fclose(f);
	if (!ok)
		return fail("the recording is truncated or damaged");
	return true;
}


size_t
Recording::_SampleIndex(double get) const
{
	// Last sample at or before `get`.
	auto it = std::upper_bound(samples.begin(), samples.end(), get,
		[](double g, const RecSample& s) { return g < s.get; });
	if (it == samples.begin())
		return 0;
	return size_t(it - samples.begin()) - 1;
}


RecSample
Recording::At(double get) const
{
	if (samples.empty())
		return RecSample();
	if (get <= samples.front().get)
		return samples.front();
	if (get >= samples.back().get)
		return samples.back();
	size_t i = _SampleIndex(get);
	const RecSample& a = samples[i];
	const RecSample& b = samples[i + 1];
	double h = b.get - a.get;
	double t = h > 0 ? (get - a.get) / h : 0;
	RecSample out = a;
	out.get = get;
	out.moon = Hermite(a.moon, a.moonV, b.moon, b.moonV, h, t);
	out.moonV = a.moonV + (b.moonV - a.moonV) * t;
	out.altitude = a.altitude + (b.altitude - a.altitude) * t;
	out.speed = a.speed + (b.speed - a.speed) * t;
	for (int k = 0; k < REC_SLOTS; k++) {
		const RecVehicle& va = a.veh[k];
		const RecVehicle& vb = b.veh[k];
		RecVehicle& vo = out.veh[k];
		// Only blend between samples of the same vehicle configuration
		// (staging and docking change what's tracked).
		if (!va.Present() || !vb.Present() || va.modules != vb.modules) {
			if (va.Present())
				vo.r = va.r + va.v * (get - a.get);
			continue;
		}
		vo.r = Hermite(va.r, va.v, vb.r, vb.v, h, t);
		vo.v = va.v + (vb.v - va.v) * t;
		vo.q = Slerp(va.q, vb.q, t);
	}
	return out;
}


const RecDisplay*
Recording::DisplayAt(bool fromLGC, double get) const
{
	const std::vector<RecDisplay>& d = fromLGC ? lgc : cmc;
	auto it = std::upper_bound(d.begin(), d.end(), get,
		[](double g, const RecDisplay& x) { return g < x.get; });
	if (it == d.begin())
		return nullptr;
	return &*(it - 1);
}


void
Recording::EventsBetween(double from, double to, std::vector<size_t>* out) const
{
	out->clear();
	auto it = std::upper_bound(events.begin(), events.end(), from,
		[](double g, const RecEvent& e) { return g < e.get; });
	for (; it != events.end() && it->get <= to; ++it)
		out->push_back(size_t(it - events.begin()));
}


int
Recording::PhaseAt(double get) const
{
	if (samples.empty())
		return 0;
	return samples[_SampleIndex(get)].phase;
}


const char*
RecPhaseName(int phase)
{
	static const char* names[] = {
		"Prelaunch", "Boost", "Earth parking orbit", "Translunar injection",
		"Translunar coast", "Lunar orbit", "Powered descent", "Lunar surface",
		"Lunar ascent", "Rendezvous", "Transearth coast", "Entry", "Recovery",
		"Mission lost"
	};
	if (phase < 0 || phase >= int(sizeof(names) / sizeof(names[0])))
		return "";
	return names[phase];
}

}  // namespace flight
