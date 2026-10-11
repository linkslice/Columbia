// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// A flight program: its rope image and symbol table (extracted from the
// assembler listing), plus helpers to pad-load erasable memory by name in
// the computer's fixed-point formats.

#ifndef COLUMBIA_ROPE_H
#define COLUMBIA_ROPE_H

#include "../agc/AGC.h"
#include "../sim/Vec3.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace gnc {

struct Symbol {
	bool erasable = true;
	int bank = 0;
	int offset = 0;
};

class Rope {
public:
	// Loads `<dir>/<program>.bin` and `<dir>/sym/<program>.sym`.
	bool Load(const std::string& dir, const std::string& program,
		std::string* error = nullptr);

	const std::string& Program() const { return fProgram; }
	const std::vector<uint8_t>& Image() const { return fImage; }
	bool Find(const std::string& name, Symbol* symbol) const;
	bool Has(const std::string& name) const;

	// Reads a fixed-memory constant (e.g. rope launch data) as a fraction.
	bool FixedDP(const std::string& name, double* fraction) const;

private:
	std::string fProgram;
	std::vector<uint8_t> fImage;
	std::map<std::string, Symbol> fSymbols;
};

// Fixed-point encodings used by the flight software.
uint16_t EncodeSP(double fraction);                 // B0 single precision
void EncodeDP(double fraction, uint16_t* hi, uint16_t* lo);
double DecodeSP(uint16_t word);
double DecodeDP(uint16_t hi, uint16_t lo);

// Writes values into a computer's erasable memory by symbol name.
// "value B n" in the flight documents means fraction = value * 2^n.
class PadLoader {
public:
	PadLoader(agc::Computer& computer, const Rope& rope)
		: fComputer(computer), fRope(rope) {}

	bool Octal(const std::string& name, uint16_t word, int index = 0);
	bool Single(const std::string& name, double value, int scale,
		int index = 0);
	bool Integer(const std::string& name, int value, int index = 0);
	bool Double(const std::string& name, double value, int scale,
		int index = 0);
	bool Vector(const std::string& name, const sim::Vec3& v, int scale);
	bool ReadDouble(const std::string& name, int scale, double* value,
		int index = 0) const;
	bool ReadSingle(const std::string& name, int scale, double* value,
		int index = 0) const;
	bool ReadVector(const std::string& name, int scale, sim::Vec3* v) const;
	uint16_t Word(const std::string& name, int index = 0) const;
	bool SetWord(const std::string& name, uint16_t word, int index = 0);

	// Names that were missing from the program (for diagnostics).
	const std::vector<std::string>& Missing() const { return fMissing; }

private:
	bool _Address(const std::string& name, int index, int* bank,
		int* offset) const;

	agc::Computer& fComputer;
	const Rope& fRope;
	mutable std::vector<std::string> fMissing;
};

}  // namespace gnc

#endif
