// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Rope.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace gnc {

bool
Rope::Load(const std::string& dir, const std::string& program,
	std::string* error)
{
	fProgram = program;
	fImage.clear();
	fSymbols.clear();
	std::ifstream bin(dir + "/" + program + ".bin", std::ios::binary);
	if (!bin) {
		if (error)
			*error = "missing rope " + dir + "/" + program + ".bin";
		return false;
	}
	fImage.assign(std::istreambuf_iterator<char>(bin),
		std::istreambuf_iterator<char>());
	std::ifstream sym(dir + "/sym/" + program + ".sym");
	if (!sym) {
		if (error)
			*error = "missing symbols for " + program;
		return false;
	}
	std::string line;
	while (std::getline(sym, line)) {
		std::istringstream in(line);
		std::string name, kind;
		Symbol s;
		if (in >> name >> kind >> s.bank >> s.offset) {
			s.erasable = kind == "E";
			fSymbols[name] = s;
		}
	}
	return fImage.size() == 36 * 1024 * 2;
}


bool
Rope::Find(const std::string& name, Symbol* symbol) const
{
	auto it = fSymbols.find(name);
	if (it == fSymbols.end())
		return false;
	*symbol = it->second;
	return true;
}


bool
Rope::Has(const std::string& name) const
{
	return fSymbols.count(name) != 0;
}


bool
Rope::FixedDP(const std::string& name, double* fraction) const
{
	Symbol s;
	if (!Find(name, &s) || s.erasable)
		return false;
	// File order is banks 2, 3, 0, 1, 4, 5...
	int block = s.bank < 4 ? (s.bank + 2) % 4 : s.bank;
	auto word = [&](int offset) {
		size_t at = (size_t(block) * 1024 + offset) * 2;
		if (at + 1 >= fImage.size())
			return uint16_t(0);
		return uint16_t(((fImage[at] << 8) | fImage[at + 1]) >> 1);
	};
	*fraction = DecodeDP(word(s.offset), word(s.offset + 1));
	return true;
}


// ---------------------------------------------------------------------------
// Encodings


uint16_t
EncodeSP(double fraction)
{
	double m = std::fabs(fraction);
	if (m > 1)
		m = 1;
	uint16_t mag = uint16_t(std::lround(m * 16384.0));
	if (mag > 037777)
		mag = 037777;
	return fraction < 0 ? uint16_t(~mag & 077777) : mag;
}


void
EncodeDP(double fraction, uint16_t* hi, uint16_t* lo)
{
	double m = std::fabs(fraction);
	if (m >= 1)
		m = 1 - 1e-12;
	uint64_t mag = uint64_t(std::llround(m * double(1 << 28)));
	if (mag > (uint64_t(1) << 28) - 1)
		mag = (uint64_t(1) << 28) - 1;
	uint16_t h = uint16_t((mag >> 14) & 037777);
	uint16_t l = uint16_t(mag & 037777);
	if (fraction < 0) {
		h = uint16_t(~h & 077777);
		l = uint16_t(~l & 077777);
	}
	*hi = h;
	*lo = l;
}


double
DecodeSP(uint16_t word)
{
	return agc::ToInt15(word) / 16384.0;
}


double
DecodeDP(uint16_t hi, uint16_t lo)
{
	return agc::ToInt15(hi) / 16384.0 + agc::ToInt15(lo) / (16384.0 * 16384.0);
}


// ---------------------------------------------------------------------------
// PadLoader


bool
PadLoader::_Address(const std::string& name, int index, int* bank,
	int* offset) const
{
	Symbol s;
	if (!fRope.Find(name, &s) || !s.erasable) {
		fMissing.push_back(name);
		return false;
	}
	int linear = s.bank * 256 + s.offset + index;
	*bank = linear / 256;
	*offset = linear % 256;
	return true;
}


bool
PadLoader::Octal(const std::string& name, uint16_t word, int index)
{
	int b, o;
	if (!_Address(name, index, &b, &o))
		return false;
	fComputer.WriteErasable(b, o, word);
	return true;
}


bool
PadLoader::SetWord(const std::string& name, uint16_t word, int index)
{
	return Octal(name, word, index);
}


uint16_t
PadLoader::Word(const std::string& name, int index) const
{
	int b, o;
	if (!_Address(name, index, &b, &o))
		return 0;
	return fComputer.ReadErasable(b, o);
}


bool
PadLoader::Single(const std::string& name, double value, int scale,
	int index)
{
	return Octal(name, EncodeSP(std::ldexp(value, scale)), index);
}


bool
PadLoader::Integer(const std::string& name, int value, int index)
{
	uint16_t w = value < 0 ? uint16_t(~uint16_t(-value) & 077777)
		: uint16_t(value & 037777);
	return Octal(name, w, index);
}


bool
PadLoader::Double(const std::string& name, double value, int scale,
	int index)
{
	uint16_t hi, lo;
	EncodeDP(std::ldexp(value, scale), &hi, &lo);
	return Octal(name, hi, index) && Octal(name, lo, index + 1);
}


bool
PadLoader::Vector(const std::string& name, const sim::Vec3& v, int scale)
{
	return Double(name, v.x, scale, 0) && Double(name, v.y, scale, 2)
		&& Double(name, v.z, scale, 4);
}


bool
PadLoader::ReadDouble(const std::string& name, int scale, double* value,
	int index) const
{
	int b, o, b2, o2;
	if (!_Address(name, index, &b, &o) || !_Address(name, index + 1, &b2, &o2))
		return false;
	*value = std::ldexp(DecodeDP(fComputer.ReadErasable(b, o),
		fComputer.ReadErasable(b2, o2)), -scale);
	return true;
}


bool
PadLoader::ReadSingle(const std::string& name, int scale, double* value,
	int index) const
{
	int b, o;
	if (!_Address(name, index, &b, &o))
		return false;
	*value = std::ldexp(DecodeSP(fComputer.ReadErasable(b, o)), -scale);
	return true;
}


bool
PadLoader::ReadVector(const std::string& name, int scale, sim::Vec3* v) const
{
	return ReadDouble(name, scale, &v->x, 0) && ReadDouble(name, scale, &v->y, 2)
		&& ReadDouble(name, scale, &v->z, 4);
}

}  // namespace gnc
