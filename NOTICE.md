# Third-party material

Columbia's own code is MIT licensed (see `LICENSE`). It contains no code
from other projects. It does contain the following material that came from
elsewhere, none of which carries terms that conflict with the MIT license.

## The Apollo flight software (public domain)

`rope/*.bin`, `rope/sym/*.sym` and `core/agc/Comanche055Rope.cpp` are the
Apollo Guidance Computer programs Colossus, Comanche, Artemis, Sundance and
Luminary, written by the MIT Instrumentation Laboratory under contract to
NASA. They are in the public domain.

The images were assembled from the transcriptions and reconstructions of
the [Virtual AGC](https://github.com/virtualagc/virtualagc) project; the
symbol tables are the assembler's listing of the same programs. Only the
assembled programs are included: none of Virtual AGC's GPL-licensed tools
(yaAGC, yaYUL, oct2bin) or their code is in Columbia. `tools/verify_rope.sh`
downloads and runs yaYUL to check the images, at verification time only.
See `rope/PROVENANCE.md` for which program is which and how each was
checked.

## Mission pad loads (NASA mission data)

`core/gnc/MissionPads.cpp` is a table of numbers: the constants loaded into
the CM and LM computers before each of Apollo 7-12 (DAP data, engine
parameters, entry and landing targets). They are NASA mission data. The
values were read, with `tools/extract_padloads.py`, from the computer
memory images in the mission scenarios of the
[NASSP](https://github.com/orbiternassp/NASSP) project (Project Apollo for
Orbiter), whose authors reconstructed them from NASA documentation; the
names come from the flight programs' own listings. No NASSP source code or
scenario files are included. NASSP itself is GPL-2.0; with thanks to its
authors for the reconstruction work.

## Coastlines (public domain)

`ui/Coastline.cpp` is generated from the
[Natural Earth](https://www.naturalearthdata.com) 1:110m coastline data,
which is in the public domain.

## Haiku

The build links against Haiku's system libraries (libbe, libtracker,
libdevice, libscreensaver), which are MIT licensed. `tests/preview` holds a
small stand-in for parts of the Interface Kit, written for Columbia, so the
drawing code can be previewed off Haiku.
