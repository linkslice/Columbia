# Rope provenance

| File | Program | Spacecraft | SHA-256 |
|---|---|---|---|
| `Comanche055.bin` | Comanche 055 (Colossus 2A) | Apollo 11 Command Module | `2ba31de9291cd10fb351a64d261bae8514a1cb75b4651bfa6a135dfa821a2d79` |
| `Luminary099.bin` | Luminary 099 rev 1 (Luminary 1A) | Apollo 11 Lunar Module | `1f5326e038de5b741b2f27b01ec949dbd688cf1906994e997402587c8628f40e` |

Both programs are public domain. Columbia flies the Command Module, so only
Comanche 055 is used today (it is compiled into the app from
`core/agc/Comanche055Rope.cpp`). Luminary 099 is included for a future LM.

## Which transcription, and why

The images were assembled with yaYUL from the **Virtual AGC** repository
(`github.com/virtualagc/virtualagc`), not from the popular
`chrislgarry/Apollo-11` GitHub mirror. Checked on 2026-10-10:

| Source | Assembles with current yaYUL | Matches the octal rope listing |
|---|---|---|
| Virtual AGC `Comanche055` | yes, 0 errors | **bit for bit** |
| Virtual AGC `Luminary099` | yes, 0 errors | **bit for bit**, and also matches Don Eyles's independently transcribed `AP11ROPE` listing |
| chrislgarry `Comanche055` | no (1 fatal error) | — |
| chrislgarry `Luminary099` | no (2,572 fatal errors) | — |

The octal listings are transcriptions of the printed rope contents (the
numbers actually woven into the core rope), made separately from the source
listings, so agreement between the two is strong evidence the source text is
right. The memory bank checksums ("bugger words") also match the printouts.

The `Luminary099` source is revision 1, the one that flew. Virtual AGC also
reconstructs revision 0 (`LMY99R0`), which was superseded days before launch.

## Reproducing

```sh
tools/verify_rope.sh
```

It clones Virtual AGC, builds yaYUL and oct2bin, assembles both programs, and
compares each against its octal listing and against the files here.

## File layout

The standard Virtual AGC `.bin` layout: 36 banks of 1024 words, in the
order 2, 3, 0, 1, 4, 5, ... 43. Each word is 16 bits big-endian with the
15-bit value in bits 15..1 (bit 0, parity, is zero).
