# Design: Synchronous rewrite of the I2C register map (`registers.v`)

**Date:** 2026-06-15
**Repo:** openmotion-camera-fpga (seed_driver / LCMXO2-2000HC)
**Status:** Approved design — pending implementation plan

## Problem

`src/registers.v` is the I2C-accessible register map for the seed laser driver. It
behaves as a pseudo dual-port store: an I2C "port" (byte read/write through a shared
address pointer) and a parallel read port that fans every field out to the datapath
modules (`dds_gain_control`, `dds_control_interface`, `adc_control`).

In hardware it exhibits all four of these symptoms:

1. **Lost/dropped updates** — two register writes close together (e.g. DDS gain + CW
   gain, or successive `modulate_frequency` bytes) sometimes don't all take effect.
2. **Read returns wrong byte** — reading a field back over I2C can give a stale or
   corrupted value, notably for multi-byte fields.
3. **Value glitches / overwritten** — a field can be clobbered by internal logic vs an
   I2C write happening together (e.g. `control`).
4. **Addressing / sync issues** — the wrong register is intermittently read/written, as
   if the address pointer lands on the wrong location.

### Root cause

This is a **clocking** problem, not a memory-array problem. The design has no inferred
RAM or EBR anywhere; every field is a scalar config register held in flip-flops. The
file currently has five `always` blocks triggered on **five different edges**:

- `posedge clk` (write block, read block)
- `negedge clk` (start-address routine)
- `posedge data_vld_dly`, `posedge start`, `posedge stop` (address-increment FSM)
- `negedge SCL` + assorted edges (stretch logic)

Using data/control signals **as clocks** (the address FSM clocked on `data_vld_dly`,
`start`, `stop`) is what produces the addressing/sync metastability and the cross-block
races. The shared `update_count` register and the `[23:0]` slice typo produce the rest.

A literal dual-port EBR was considered and rejected: every field needs parallel,
same-cycle access by the datapath, which a 2-port RAM cannot provide (one address per
port per cycle) — you would have to fan values back out into flip-flops anyway,
rebuilding the register file on top of the RAM. EBRs are for buffers/arrays; this
design has none. The right *concept* is independent collision-free read/write ports;
the right *implementation* is a clean synchronous register file.

## Goals

- Eliminate all four symptom classes at the root.
- Keep the `registers` module port list **byte-for-byte identical** so `top.v`,
  `i2c_slave_top.v`, and the datapath modules are untouched.
- No change to the synthesized source set beyond the rewritten `registers.v`
  (testbench is sim-only, not added to `seed_driver.ldf`).

## Non-goals

- Clock stretching. `stretch_on` is already hardwired to `0` today (the `stretch_test`
  define is never set), so stretching is effectively disabled. The new code keeps
  `stretch_on = 0` and drops the dead SCL-domain stretch logic. Out of scope to make
  it work.
- Timing closure / place-and-route in Diamond (cannot be run in this environment).
- Any change to the register address map, field semantics, or the I2C protocol.

## Design

Rewrite the internals of `src/registers.v` so **all logic runs in one synchronous
`clk` (buf_clk) domain**. `start`, `stop`, and `data_vld` are treated as synchronous
strobes (they are already clean `i_sys_clk`-domain single-cycle pulses out of the I2C
controller). `r_w` and `i2c_to_data` originate in the slow SCL domain and are sampled
on the `data_vld` strobe — safe because SCL (≤400 kHz) is far slower than buf_clk
(25 MHz+) and both are stable for many buf_clk cycles around the strobe.

### Internal blocks (all `@(posedge clk or negedge rstn)`)

| Block | Purpose |
|---|---|
| Strobe capture | register `start`/`stop`/`data_vld`; derive 1-cycle `data_vld_rise` |
| Address pointer FSM | synchronous: `start` resets transaction state; first written byte loads the register index; each subsequent byte auto-increments — replicating the **exact** byte→address mapping of the current FSM |
| Write port | one `case(ptr)` gated by a write strobe (`data_vld_rise & ~r_w & index_loaded`); fixes the `modulate_frequency_temp[23:0]` slice → `[23:16]` |
| Read-back mux | one `case(ptr)` → `data_out`; address map kept identical to the write side |
| Update one-shots | **per-field independent** one-shot pulse generators for `dds_gain_update`, `cw_gain_update`, `dds_current_limit_update`, `cw_current_limit_update`, `dds_mon_current_limit_update`, `cw_mon_current_limit_update`; removes the shared `update_count` |

### Symptom → fix mapping

- **Lost/dropped updates** → each `*_update` gets its own one-shot; no shared counter to
  truncate a second update. Back-to-back DDS+CW gain writes produce both DAC updates.
- **Read returns wrong byte** → single read mux in the pointer's clock domain + the
  `[23:0]`→`[23:16]` slice fix; multi-byte fields read back correctly.
- **Value glitches/overwritten** → `control`'s I2C write and its auto-clear live in one
  synchronous block with defined priority (I2C write wins and restarts the one-shot).
  Existing semantics preserved: `modulate_configurate = control[0]` remains a transient
  pulse; `mcu_gpio = control[15]`.
- **Addressing/sync** → the async `posedge start/stop/data_vld_dly` and `negedge clk`
  FSMs become one synchronous pointer FSM. This removes the metastability root cause.

### Behavior preservation

The address map (write cases `0x0`–`0x23`, read cases `0x0`–`0x21`), reset/default
values (e.g. `cw_gain = 16'h07f5`, `dds_current_limit = 16'h03dc`, `cw_current_limit =
16'h0800`, `modulate_frequency_temp = 28'h012000`), the power-on init pulse that fires
`cw_gain_update` after `init_count`, and the `control` countdown duration are carried
over unchanged except where a change is the explicit fix (the `[23:16]` slice and the
removal of the shared counter).

## Verification

**Tooling:** Icarus Verilog (`iverilog -g2012`, then `vvp`), per the repo's stated FPGA
sim flow. First implementation step confirms `iverilog` is on PATH; if absent, flag
before writing tests.

**Testbench:** new `src/tb_registers.v` instantiating the real `i2c_slave_top`
(exercising `registers` + `i2cslave_controller_top` together) driven by the existing
`i2c_master` BFM in `src/master.v`. Sim-only; **not** added to `seed_driver.ldf`.

**Test cases (TDD — written to fail against current `registers.v`, then pass after the
rewrite):**

1. Round-trip each field: write, read back, assert (directly catches the
   `modulate_frequency` slice bug).
2. Back-to-back updates: write DDS gain hi-byte then CW gain hi-byte in quick
   succession; assert **both** `dds_gain_update` and `cw_gain_update` strobe. Lost-update
   reproducer.
3. Multi-byte field: write all four `modulate_frequency` bytes, read back, assert the
   full 28-bit value.
4. Auto-increment burst: sequential read across the status/revision/ID block; assert
   each byte lands on the right address.
5. `control` one-shot: write `control`, assert `modulate_configurate` pulses and
   `control` returns to 0; immediately write again to confirm no stuck/dropped state.

**Caveat (recorded honestly):** simulation proves the *functional* fixes (1–5). The
**metastability** portion of the addressing/sync symptom is a real-silicon async-clocking
effect and will not reliably reproduce in a zero-delay sim; that fix is justified by the
async→sync restructuring itself, not by a sim waveform.

## Risks

- The address-pointer FSM must reproduce the current byte→address mapping exactly
  (index byte vs first data byte vs read auto-increment). This is the highest-risk part;
  test cases 1, 3, and 4 are the guard.
- `r_w`/`i2c_to_data` cross-domain sampling relies on SCL ≪ buf_clk; documented and true
  for this design, but noted for any future faster-bus change.

## Files

- `src/registers.v` — rewritten internals, identical ports.
- `src/tb_registers.v` — new, sim-only testbench.
- `seed_driver.ldf` — unchanged.
