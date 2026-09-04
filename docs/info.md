# RC5-72 Iterative Cracker Core

## How it works

Tests one 72-bit RC5 key at a time against a target ciphertext, over a
5-signal SPI slave interface. Deliberately area-optimized rather than
throughput-optimized: a single shared datapath (one adder/rotator pair)
is reused sequentially across the 78-step key schedule and the 12
encryption rounds, instead of the fully-pipelined, one-key-per-clock
design this project originally built and proved out on an ECP5 FPGA.
That earlier design ran roughly 27,000 flip-flops - far more than a
small shuttle tile budget allows. This core uses roughly 1,675, at the
cost of taking ~93 clock cycles to test each key instead of 1.

### Protocol

Request (31 bytes, PC/controller -> this core):
- byte 0: sync (0xAA)
- byte 1: cmd (0x01)
- bytes 2-10: base key (9 bytes, little-endian)
- bytes 11-14: count (32-bit, how many keys to test, starting at base key)
- bytes 15-18: plaintext half A
- bytes 19-22: plaintext half B
- bytes 23-26: target ciphertext half A
- bytes 27-30: target ciphertext half B

Response (this core -> PC/controller):
- byte 0: sync (0x55)
- byte 1: status - 0x00 (no match/no partial match), 0x01 (full match),
  0x02 (partial match only - low half of the ciphertext matched, high
  half didn't, at least once in the batch)
- if 0x01: bytes 2-10 = the matching key, bytes 11-14/15-18 = its
  ciphertext (both halves)
- if 0x02: bytes 2-10 = the most recent partial-match key, bytes 11-14 =
  how many partial matches occurred in the batch
- if 0x00: nothing further

A full match ends the batch early (does not keep testing the remaining
keys in `count`) - a deliberate choice, since every cycle matters more
on this core than it did on the original, much faster FPGA pipeline.

## How to test

Drive the SPI request byte sequence above via `ui_in[2:0]`
(sclk/mosi/cs_n), read the response back via `uo_out[1:0]`
(miso/ready). The algorithm and this exact cycle-by-cycle datapath
structure were verified against a bit-exact Python reference model
before the RTL was written, then confirmed correct in RTL simulation
against three known key/plaintext/ciphertext triples covering all three
response types (no-match, partial-match, full-match) - see the project's
own test vectors for a ready-made starting point.

## External hardware

None required beyond an SPI-capable controller (microcontroller, or
another FPGA/board acting as SPI master) driving the 5-pin interface
above.
