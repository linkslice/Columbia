# Rope provenance

Every program here is public domain (work of the US government and its
contractors, MIT Instrumentation Laboratory). Each `.bin` was assembled with
yaYUL from the **[Virtual AGC](https://www.ibiblio.org/apollo/)** sources
([github.com/virtualagc/virtualagc](https://github.com/virtualagc/virtualagc)),
the work of Ron Burkey and the project's many contributors;
each `sym/*.sym` is the symbol table from the same assembly, which Columbia
uses to find erasable locations by name (pad loads, the DSKY, diagnostics).

| File | Flies | Module | Octal listing | SHA-256 |
|---|---|---|---|---|
| `Colossus237.bin` | Apollo 7 (stand-in for Sundisk 282), Apollo 8 | CM | — | `01b6a1173dab6668cbfd36232600917c48727b8d0bcd0d4171747628f370ccac` |
| `Colossus249.bin` | Apollo 9 | CM | matches | `9edf37e11a5a4a38f7a72ea67ae7b6e1c357d83aadab756d8483c251721707b4` |
| `Sundance306ish.bin` | Apollo 9 | LM | — | `5cac5b525586b7ce20391b3827af81faf69d98f66d992f7c40d5988b9bbba538` |
| `Comanche045.bin` | Apollo 10 | CM | — | `acae7799dca46b45db746acc4eeea4a8c80778e74eb076a9460f57f5974e092b` |
| `LUM69R2.bin` | Apollo 10 | LM | — | `9e1a540cd6c710e806e78e10dcd84a3426d98b4273f926b946497d820886c350` |
| `Comanche055.bin` | Apollo 11 | CM | matches | `2ba31de9291cd10fb351a64d261bae8514a1cb75b4651bfa6a135dfa821a2d79` |
| `Luminary099.bin` | Apollo 11 | LM | matches | `1f5326e038de5b741b2f27b01ec949dbd688cf1906994e997402587c8628f40e` |
| `Comanche067.bin` | Apollo 12 | CM | — | `ff9aad111257cd26304db8c4b77c4d22e49bd68a0694a5e8f456aa5d606a9829` |
| `Luminary116.bin` | Apollo 12 | LM | matches | `97e3889303728577aa13bec11117a9ff2469bd6f5a74a8a98af841b51ba1fba4` |
| `Comanche072.bin` | Apollo 13, Apollo 14 (stand-in for Comanche 108) | CM | — | `c70e730536728b9b29fe67f0ae65d14cab9b9db081c2a07aaa4c5803c7f91d10` |
| `LM131R1.bin` | Apollo 13 | LM | — | `f333b7de10113b4c211cda82a573f01e88cea69899dbb5a4ec47d104d041c4f1` |
| `Luminary178.bin` | Apollo 14 | LM | — | `f45d747109fcea701b915857b649dd3ad56f9c1693da3144e49a91268684e4db` |
| `Artemis072.bin` | Apollo 15-17 | CM | matches | `4f875ead7787e0e2c8119a75d3eade81308a6a6da6310c3bd3c6d73d86e3378a` |
| `Luminary210.bin` | Apollo 15-17 | LM | matches | `416cb5504424b6941614fd974a8c950c6217ae98ae01331ee0c759ab42561c76` |

"Octal listing" is the independent check: a transcription of the printed
rope contents (the numbers actually woven into the core rope), made
separately from the source listings. Where Virtual AGC has one, the
assembled image agrees with it bit for bit, which is strong evidence the
source text is right. The rest assemble with no errors but have nothing
independent to compare against; some (`Sundance306ish`, `LUM69R2`) are
Virtual AGC reconstructions of versions whose listings don't survive.

Two missions use the nearest surviving program in place of the one that
flew: Apollo 7 flies Colossus 237 (Sundisk 282 isn't available) and Apollo 14's CM
flies Comanche 072 (Comanche 108 isn't available). The differences are small for
what the simulator does, but they are there.

Checked against Virtual AGC revision `1b5d647` (2026-10-10).

## Why not the `chrislgarry/Apollo-11` mirror

The popular GitHub mirror of the Apollo 11 code doesn't assemble with
current yaYUL (Comanche055: 1 fatal error, Luminary099: 2,572), so it was not
used. Virtual AGC's `Luminary099` is revision 1, the one that flew; it also
matches the separately transcribed `AP11ROPE` octal listing.

## Reproducing

```sh
tools/verify_rope.sh
```

It clones Virtual AGC, builds yaYUL and oct2bin, assembles every program
listed here, and compares each against its octal listing (where there is
one) and against the file here. Virtual AGC's tools are GPL and are only
used at verification time; nothing from them is linked into Columbia.

## File layout

The standard Virtual AGC `.bin` layout: 36 banks of 1024 words, in the
order 2, 3, 0, 1, 4, 5, ... 43. Each word is 16 bits big-endian with the
15-bit value in bits 15..1 (bit 0, parity, is zero).
