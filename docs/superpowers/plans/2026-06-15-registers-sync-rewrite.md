# Registers Synchronous Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the internals of `src/registers.v` as a single-clock-domain synchronous register file that eliminates the I2C register-map read/write collisions, while keeping the module's external interface and the observable register-map behavior identical.

**Architecture:** Collapse the current five-edge clocking (`posedge clk`, `negedge clk`, `posedge data_vld_dly/start/stop`, `negedge SCL`) into one `posedge clk` domain. Treat `start`/`stop`/`data_vld` as synchronous strobes, 2-FF synchronize `r_w`, drive one write port + one read-back mux from a synchronous address pointer, and replace the shared `update_count` with per-field one-shots. Behavior is locked to the *current* design via a characterization test before the rewrite.

**Tech Stack:** Verilog-2001 (design) + SystemVerilog testbench (`logic`, queues), Icarus Verilog (`iverilog -g2012` / `vvp`), existing `i2c_master` BFM in `src/master.v`.

---

## Critical constraint: preserve the current byte→address mapping

The host SDK is calibrated to the *current* hardware behavior. The exact mapping of "first data byte after the index" to a register address is timing-dependent in the current FSM and **must be measured, not assumed**. Task 2 measures it; Task 4 matches it. Do not skip Task 2.

## File structure

- `src/registers.v` — **rewrite internals**, identical port list. Single responsibility: I2C register map.
- `src/tb_registers.v` — **new, sim-only** testbench. Not added to `seed_driver.ldf`.
- `sim/run.sh` (or `.ps1`) — **new, optional** convenience wrapper for compile+run.
- `seed_driver.ldf` — **unchanged** (verify at the end it still lists the same sources).

Reference facts (from current source, do not re-derive):
- Slave address: `i2cslave_controller` param `i_slave_addr = 10'b11_1100_0001`, 7-bit mode → **`7'h41`**.
- `data_vld = o_wr_done | o_data_valid`; `start`/`stop`/`data_vld` are clean 1-cycle `clk`-domain pulses; `r_w = ~o_tx_status` originates in the SCL domain (`r_w=0` write, `r_w=1` read).
- Write address map (current): `0x0/0x1` modulate_phrase; `0x2/0x3` dds_gain (hi triggers update); `0x4/0x5` cw_gain; `0x6/0x7` dds_current_limit; `0x8/0x9` cw_current_limit; `0xA..0xD` modulate_frequency; `0x20/0x21` static_control; `0x22/0x23` control.
- Read address map adds: `0xE/0xF` adc_current_data; `0x10/0x11` adc_voltage_data; `0x12` status; `0x13` revision; `0x14` minor; `0x15` major; `0x16` ID.
- Reset defaults: dds_gain `0`, cw_gain `0x07f5`, dds_current_limit `0x03dc`, cw_current_limit `0x0800`, modulate_frequency_temp `0x00012000`, control/static_control `0`.
- Known bug to fix: write case `0xC` is `modulate_frequency_temp[23:0]` (should be `[23:16]`) at `src/registers.v:183`.

---

### Task 0: Toolchain gate

**Files:** none (environment only)

- [ ] **Step 1: Check for iverilog**

Run (PowerShell): `Get-Command iverilog -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source`
Expected: a path. If empty, install it.

- [ ] **Step 2: Install if missing**

Run one of:
`winget install --id IcarusVerilog.IcarusVerilog -e` (preferred), or `choco install iverilog -y`.
Then open a fresh shell and re-run Step 1. Expected: a path is printed, and `iverilog -V` prints a version banner.

- [ ] **Step 3: STOP and report if install is not possible**

If iverilog cannot be installed in this environment, stop here and report to the user — the rest of the plan depends on simulation. Do not proceed with blind RTL edits.

---

### Task 1: Testbench harness that drives the *current* design

**Files:**
- Create: `src/tb_registers.v`
- (uses existing `src/master.v`, `src/i2c_slave_top.v`, `src/i2cslave_controller_top.v`, `src/i2cslave_controller.v`, `src/registers.v`, `src/filter.v`)

- [ ] **Step 1: Write the harness + one smoke test**

Create `src/tb_registers.v`:

```verilog
`timescale 1ns / 1ps
// Sim-only testbench for the I2C register map. NOT part of seed_driver.ldf.
module tb_registers;

    localparam [6:0] SLAVE = 7'h41;

    reg         clk = 1'b0;
    reg         rstn = 1'b0;
    wire        scl, sda;

    // Stub datapath inputs into the register map
    reg  [15:0] adc_voltage_data = 16'h1234;
    reg  [15:0] adc_current_data = 16'h5678;
    reg  [7:0]  monitor_status   = 8'hA5;
    reg  [7:0]  status           = 8'h0F;
    reg  [7:0]  revision = 8'h01, minor = 8'h01, major = 8'h00, ID = 8'h01;

    // Register-map outputs we observe
    wire [15:0] dds_gain, cw_gain, dds_current_limit, cw_current_limit;
    wire [27:0] modulate_frequency;
    wire [13:0] modulate_phrase;
    wire        dds_gain_update, cw_gain_update;
    wire        dds_current_limit_update, cw_current_limit_update;
    wire        dds_mon_current_limit_update, cw_mon_current_limit_update;
    wire [15:0] dds_mon_current_limit, cw_mon_current_limit;
    wire [15:0] control, static_control;

    integer errors = 0;
    // Update-pulse counters (catch lost/dropped updates)
    integer dds_gain_update_cnt = 0, cw_gain_update_cnt = 0;

    // 25 MHz system clock (40 ns)
    always #20 clk = ~clk;

    // Slow I2C so SCL events are far slower than clk (validates sync sampling)
    i2c_master #(.i2c_delay(2000)) MASTER (.SDA(sda), .SCL(scl), .RST(1'b0));

    i2c_slave_top DUT (
        .rstn(rstn), .clk(clk), .scl(scl), .sda(sda),
        .adc_voltage_data(adc_voltage_data), .adc_current_data(adc_current_data),
        .monitor_status(monitor_status), .status(status),
        .revision(revision), .minor(minor), .major(major), .ID(ID),
        .dds_gain(dds_gain), .cw_gain(cw_gain),
        .dds_current_limit(dds_current_limit), .cw_current_limit(cw_current_limit),
        .modulate_frequency(modulate_frequency), .modulate_phrase(modulate_phrase),
        .dds_gain_update(dds_gain_update), .cw_gain_update(cw_gain_update),
        .dds_current_limit_update(dds_current_limit_update),
        .cw_current_limit_update(cw_current_limit_update),
        .dds_mon_current_limit_update(dds_mon_current_limit_update),
        .cw_mon_current_limit_update(cw_mon_current_limit_update),
        .dds_mon_current_limit(dds_mon_current_limit),
        .cw_mon_current_limit(cw_mon_current_limit),
        .control(control), .static_control(static_control)
    );

    // Count update pulses across the whole run
    always @(posedge clk) begin
        if (dds_gain_update) dds_gain_update_cnt = dds_gain_update_cnt + 1;
        if (cw_gain_update)  cw_gain_update_cnt  = cw_gain_update_cnt  + 1;
    end

    task check8(input [127:0] name, input [7:0] got, input [7:0] exp);
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL %0s: got %02h exp %02h", name, got, exp);
            end else $display("PASS %0s = %02h", name, got);
        end
    endtask

    task check16(input [127:0] name, input [15:0] got, input [15:0] exp);
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL %0s: got %04h exp %04h", name, got, exp);
            end else $display("PASS %0s = %04h", name, got);
        end
    endtask

    logic [7:0] wbuf [$];
    logic [7:0] rbuf [$];

    initial begin
        $dumpfile("tb_registers.vcd");
        $dumpvars(0, tb_registers);
        // reset
        rstn = 1'b0;
        repeat (10) @(posedge clk);
        rstn = 1'b1;
        repeat (10) @(posedge clk);

        // Smoke test: write static_control (0x20/0x21) and read it back.
        wbuf = '{8'hCD, 8'hAB};                 // 0x20<=0xCD, 0x21<=0xAB
        MASTER.i2c_write(SLAVE, 8'h20, 2, wbuf);
        repeat (50) @(posedge clk);
        check16("static_control_reg", static_control, 16'hABCD);

        rbuf = '{8'h00, 8'h00};
        MASTER.i2c_read(SLAVE, 8'h20, 2, rbuf);
        check8("static_control_rd0", rbuf[0], 8'hCD);
        check8("static_control_rd1", rbuf[1], 8'hAB);

        $display("ERRORS=%0d", errors);
        $finish;
    end

endmodule
```

- [ ] **Step 2: Compile and run against the current design**

Run (from repo root):
```
iverilog -g2012 -o sim_reg.out -s tb_registers src/tb_registers.v src/master.v src/i2c_slave_top.v src/i2cslave_controller_top.v src/i2cslave_controller.v src/registers.v src/filter.v
vvp sim_reg.out
```
Expected: it compiles and runs to `$finish`. Record whether `static_control` round-trips. If the smoke test fails to even communicate (no PASS lines), debug the harness (clock ratio, address, BFM wiring) before continuing — this validates the harness, not the DUT.

- [ ] **Step 3: Commit the harness**

```
git add src/tb_registers.v
git commit -m "test: add iverilog testbench harness for I2C register map"
```

---

### Task 2: Characterize the current byte→address mapping (golden behavior)

**Files:** Modify `src/tb_registers.v` (add a characterization block, temporary `$display`s)

- [ ] **Step 1: Add a mapping probe**

In `tb_registers.v`, before `$finish`, add:

```verilog
        // ---- Characterization: where does the first data byte land? ----
        // Write index=0x02 with three data bytes, then read 0x02..0x04.
        wbuf = '{8'h11, 8'h22, 8'h33};
        MASTER.i2c_write(SLAVE, 8'h02, 3, wbuf);
        repeat (50) @(posedge clk);
        $display("CHAR dds_gain (0x02/0x03) = %04h", dds_gain);
        rbuf = '{8'h00, 8'h00, 8'h00, 8'h00};
        MASTER.i2c_read(SLAVE, 8'h02, 4, rbuf);
        $display("CHAR read 0x02=%02h 0x03=%02h 0x04=%02h 0x05=%02h",
                 rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
```

- [ ] **Step 2: Run and record the golden mapping**

Run the compile+run commands from Task 1 Step 2.
Record the printed `CHAR` lines. Determine concretely:
- Does data byte `0x11` land at address `0x02` (mapping = "first data at index") or at `0x03` (off-by-one)?
- Confirm read-back addressing matches write addressing.

Write the observed mapping into a comment block at the top of `tb_registers.v` titled `// GOLDEN MAPPING (measured <date>):` so Task 4 has an unambiguous target. **This measured mapping is the contract the rewrite must satisfy.**

- [ ] **Step 3: Commit the characterization**

```
git add src/tb_registers.v
git commit -m "test: characterize current register-map byte->address mapping"
```

---

### Task 3: Lock in regression + bug-reproducer tests (run against current design)

**Files:** Modify `src/tb_registers.v`

- [ ] **Step 1: Add round-trip, multi-byte, update-count, and control tests**

Replace the smoke/characterization `initial` body's test section (keep reset + harness) with the full suite below. Use the addressing convention you measured in Task 2 (here written assuming "first data byte lands at index"; **adjust offsets to your measured golden mapping**):

```verilog
        // T1: dds_gain round-trip + update fires
        wbuf = '{8'hAD, 8'hDE};                  // 0x02<=AD, 0x03<=DE
        dds_gain_update_cnt = 0;
        MASTER.i2c_write(SLAVE, 8'h02, 2, wbuf);
        repeat (50) @(posedge clk);
        check16("T1 dds_gain", dds_gain, 16'hDEAD);
        if (dds_gain_update_cnt < 1) begin
            errors = errors + 1; $display("FAIL T1 dds_gain_update did not fire");
        end else $display("PASS T1 dds_gain_update fired %0d", dds_gain_update_cnt);

        // T2: back-to-back DDS gain then CW gain -> BOTH updates must fire
        dds_gain_update_cnt = 0; cw_gain_update_cnt = 0;
        wbuf = '{8'h01, 8'h02};                  // dds_gain <= 0x0201
        MASTER.i2c_write(SLAVE, 8'h02, 2, wbuf);
        wbuf = '{8'h03, 8'h04};                  // cw_gain  <= 0x0403
        MASTER.i2c_write(SLAVE, 8'h04, 2, wbuf);
        repeat (50) @(posedge clk);
        if (dds_gain_update_cnt < 1 || cw_gain_update_cnt < 1) begin
            errors = errors + 1;
            $display("FAIL T2 lost update: dds=%0d cw=%0d",
                     dds_gain_update_cnt, cw_gain_update_cnt);
        end else $display("PASS T2 both updates fired dds=%0d cw=%0d",
                          dds_gain_update_cnt, cw_gain_update_cnt);

        // T3: modulate_frequency 4-byte round-trip (slice-bug reproducer)
        wbuf = '{8'h78, 8'h56, 8'h34, 8'h02};    // 0xA..0xD -> 0x0_2345678 (28-bit)
        MASTER.i2c_write(SLAVE, 8'h0A, 4, wbuf);
        repeat (50) @(posedge clk);
        check16("T3 freq[15:0]",  modulate_frequency[15:0],  16'h5678);
        check16("T3 freq[27:16]", {4'h0, modulate_frequency[27:16]}, 16'h0234);

        // T4: status/ID block auto-increment read
        rbuf = '{8'h00, 8'h00, 8'h00, 8'h00, 8'h00};
        MASTER.i2c_read(SLAVE, 8'h12, 5, rbuf);  // status,rev,minor,major,ID
        check8("T4 status",   rbuf[0], status);
        check8("T4 revision", rbuf[1], 8'h01);
        check8("T4 minor",    rbuf[2], 8'h01);
        check8("T4 major",    rbuf[3], 8'h00);
        check8("T4 ID",       rbuf[4], 8'h01);

        // T5: control one-shot self-clears
        wbuf = '{8'h01, 8'h00};                  // control <= 0x0001
        MASTER.i2c_write(SLAVE, 8'h22, 2, wbuf);
        repeat (60) @(posedge clk);
        check16("T5 control_cleared", control, 16'h0000);
```

- [ ] **Step 2: Run against the CURRENT (unmodified) registers.v and record reds**

Run the compile+run commands. Expected: **T3 fails** on the current design (the `[23:0]` slice bug corrupts `freq[15:0]`). T2 may pass or fail depending on timing — record the result either way (it becomes a regression guard after the rewrite). T1/T4/T5 likely pass. Save this output as the "before" baseline.

- [ ] **Step 3: Commit the test suite**

```
git add src/tb_registers.v
git commit -m "test: register-map round-trip, lost-update, slice-bug, control tests"
```

---

### Task 4: Rewrite `registers.v` (synchronous, single domain)

**Files:**
- Modify (full replace): `src/registers.v`

- [ ] **Step 1: Replace `src/registers.v` with the synchronous implementation**

Write `src/registers.v` exactly as below. **Before running, set the write/read pointer behavior to match the Task 2 golden mapping** — the version below assumes "first data byte after the index writes at `addr=index`" and "reads start at `index`". If Task 2 measured an off-by-one, adjust the `ptr` load/increment in the pointer FSM accordingly (the single knob is whether the first post-index data byte increments `ptr` before or after the write).

```verilog
`timescale 1ns / 100ps

module registers(
    input               clk,
    input               rstn,
    input               SCL,
    output [7:0]        data_to_i2c,
    input               start,
    input               stop,
    input               data_vld,
    input               r_w,
    input [7:0]         i2c_to_data,
    output              stretch_on,
    input [15:0]        adc_current_data,
    input [15:0]        adc_voltage_data,
    input [7:0]         monitor_status,
    input [7:0]         status,
    input [7:0]         revision,
    input [7:0]         minor,
    input [7:0]         major,
    input [7:0]         ID,

    output reg [15:0]   dds_gain,
    output reg [15:0]   cw_gain,
    output reg [15:0]   dds_current_limit,
    output reg [15:0]   cw_current_limit,
    output     [27:0]   modulate_frequency,
    output     [13:0]   modulate_phrase,
    output reg          dds_gain_update,
    output reg          cw_gain_update,
    output reg          dds_current_limit_update,
    output reg          cw_current_limit_update,
    output reg          dds_mon_current_limit_update,
    output reg          cw_mon_current_limit_update,
    output reg [15:0]   dds_mon_current_limit,
    output reg [15:0]   cw_mon_current_limit,
    output reg [15:0]   control,
    output reg [15:0]   static_control
)/* synthesis syn_preserve=1 */;

    reg [31:0] modulate_frequency_temp;
    reg [15:0] modulate_phrase_temp;

    assign modulate_frequency = modulate_frequency_temp[27:0];
    assign modulate_phrase    = modulate_phrase_temp[13:0];
    assign stretch_on         = 1'b0;   // stretch disabled (matches prior behaviour)

    reg [7:0] data_out;
    assign data_to_i2c = (r_w) ? data_out : 8'h00;

    // r_w arrives from the SCL domain: 2-FF synchronize into clk
    reg r_w_meta, r_w_sync;
    always @(posedge clk or negedge rstn)
        if (!rstn) begin r_w_meta <= 1'b0; r_w_sync <= 1'b0; end
        else       begin r_w_meta <= r_w;   r_w_sync <= r_w_meta; end

    // Address-pointer FSM (single clk domain)
    reg [7:0] ptr;
    reg [7:0] start_addr;
    reg       awaiting_index;
    reg       data_vld_d;
    wire      dv_pulse = data_vld & ~data_vld_d;

    reg       wr_stb;
    reg [7:0] wr_addr;
    reg [7:0] wr_data;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ptr <= 8'h0; start_addr <= 8'h0; awaiting_index <= 1'b0;
            data_vld_d <= 1'b0; wr_stb <= 1'b0; wr_addr <= 8'h0; wr_data <= 8'h0;
        end else begin
            data_vld_d <= data_vld;
            wr_stb     <= 1'b0;
            if (start) begin
                awaiting_index <= 1'b1;
                ptr            <= start_addr;     // re-point for repeated-start reads
            end else if (dv_pulse) begin
                if (!r_w_sync) begin              // master write
                    if (awaiting_index) begin
                        start_addr     <= i2c_to_data;
                        ptr            <= i2c_to_data;
                        awaiting_index <= 1'b0;
                    end else begin
                        wr_stb  <= 1'b1;
                        wr_addr <= ptr;
                        wr_data <= i2c_to_data;
                        ptr     <= ptr + 8'h1;
                    end
                end else begin                    // master read
                    awaiting_index <= 1'b0;
                    ptr            <= ptr + 8'h1;
                end
            end
        end
    end

    // Power-on init: after settle, pulse cw_gain_update once
    reg [15:0] init_count;
    reg        enable;
    reg        init_cw_pulse;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            enable <= 1'b1; init_count <= 16'h0; init_cw_pulse <= 1'b0;
        end else begin
            init_cw_pulse <= 1'b0;
            if (enable) begin
                if (init_count > 16'he005) begin
                    enable <= 1'b0; init_cw_pulse <= 1'b1;
                end else init_count <= init_count + 16'h1;
            end
        end
    end

    // Field storage (one writer per field; control handled separately)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            modulate_phrase_temp    <= 16'h0;
            dds_gain                <= 16'h0;
            cw_gain                 <= 16'h07f5;
            dds_current_limit       <= 16'h03dc;
            cw_current_limit        <= 16'h0800;
            modulate_frequency_temp <= 32'h00012000;
            static_control          <= 16'h0;
        end else if (wr_stb) begin
            case (wr_addr)
                8'h0 : modulate_phrase_temp[7:0]      <= wr_data;
                8'h1 : modulate_phrase_temp[15:8]     <= wr_data;
                8'h2 : dds_gain[7:0]                  <= wr_data;
                8'h3 : dds_gain[15:8]                 <= wr_data;
                8'h4 : cw_gain[7:0]                   <= wr_data;
                8'h5 : cw_gain[15:8]                  <= wr_data;
                8'h6 : dds_current_limit[7:0]         <= wr_data;
                8'h7 : dds_current_limit[15:8]        <= wr_data;
                8'h8 : cw_current_limit[7:0]          <= wr_data;
                8'h9 : cw_current_limit[15:8]         <= wr_data;
                8'hA : modulate_frequency_temp[7:0]   <= wr_data;
                8'hB : modulate_frequency_temp[15:8]  <= wr_data;
                8'hC : modulate_frequency_temp[23:16] <= wr_data;  // FIX (was [23:0])
                8'hD : modulate_frequency_temp[31:24] <= wr_data;
                8'h20: static_control[7:0]            <= wr_data;
                8'h21: static_control[15:8]           <= wr_data;
                default: ;
            endcase
        end
    end

    // control: I2C write priority + restart auto-clear timer; else self-clear
    reg [3:0] ctrl_cnt;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            control <= 16'h0; ctrl_cnt <= 4'h0;
        end else begin
            if (wr_stb && wr_addr == 8'h22) begin
                control[7:0] <= wr_data; ctrl_cnt <= 4'h0;
            end else if (wr_stb && wr_addr == 8'h23) begin
                control[15:8] <= wr_data; ctrl_cnt <= 4'h0;
            end else if (control != 16'h0) begin
                if (ctrl_cnt > 4'h2) begin control <= 16'h0; ctrl_cnt <= 4'h0; end
                else ctrl_cnt <= ctrl_cnt + 4'h1;
            end
        end
    end

    // Per-field update one-shots (independent; no shared counter)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            dds_gain_update <= 1'b0; cw_gain_update <= 1'b0;
            dds_current_limit_update <= 1'b0; cw_current_limit_update <= 1'b0;
        end else begin
            dds_gain_update          <= (wr_stb && wr_addr == 8'h3);
            cw_gain_update           <= (wr_stb && wr_addr == 8'h5) || init_cw_pulse;
            dds_current_limit_update <= (wr_stb && wr_addr == 8'h7);
            cw_current_limit_update  <= (wr_stb && wr_addr == 8'h9);
        end
    end

    // monitor-limit fields: never written in this design -> constant 0
    always @(posedge clk or negedge rstn)
        if (!rstn) begin
            dds_mon_current_limit        <= 16'h0;
            cw_mon_current_limit         <= 16'h0;
            dds_mon_current_limit_update <= 1'b0;
            cw_mon_current_limit_update  <= 1'b0;
        end

    // Read-back mux (same address map as the write side)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) data_out <= 8'h0;
        else case (ptr)
            8'h0 : data_out <= modulate_phrase_temp[7:0];
            8'h1 : data_out <= modulate_phrase_temp[15:8];
            8'h2 : data_out <= dds_gain[7:0];
            8'h3 : data_out <= dds_gain[15:8];
            8'h4 : data_out <= cw_gain[7:0];
            8'h5 : data_out <= cw_gain[15:8];
            8'h6 : data_out <= dds_current_limit[7:0];
            8'h7 : data_out <= dds_current_limit[15:8];
            8'h8 : data_out <= cw_current_limit[7:0];
            8'h9 : data_out <= cw_current_limit[15:8];
            8'hA : data_out <= modulate_frequency_temp[7:0];
            8'hB : data_out <= modulate_frequency_temp[15:8];
            8'hC : data_out <= modulate_frequency_temp[23:16];
            8'hD : data_out <= modulate_frequency_temp[31:24];
            8'hE : data_out <= adc_current_data[7:0];
            8'hF : data_out <= adc_current_data[15:8];
            8'h10: data_out <= adc_voltage_data[7:0];
            8'h11: data_out <= adc_voltage_data[15:8];
            8'h12: data_out <= status;
            8'h13: data_out <= revision;
            8'h14: data_out <= minor;
            8'h15: data_out <= major;
            8'h16: data_out <= ID;
            8'h20: data_out <= static_control[7:0];
            8'h21: data_out <= static_control[15:8];
            default: data_out <= 8'h0;
        endcase
    end

endmodule
```

- [ ] **Step 2: Run the test suite against the rewrite**

Run the compile+run commands from Task 1 Step 2.
Expected: **all T1–T5 PASS and `ERRORS=0`**, including T3 (the slice fix) which was red before. If the round-trip/auto-increment addressing is off by one byte, adjust the pointer-FSM increment per the Task 2 golden mapping and re-run. Iterate until `ERRORS=0`.

- [ ] **Step 3: Commit the rewrite**

```
git add src/registers.v
git commit -m "fix: rewrite I2C register map as single-domain synchronous register file

Collapses five-edge clocking into one posedge clk domain, 2-FF
synchronizes r_w, replaces the shared update_count with per-field
one-shots, and fixes the modulate_frequency_temp[23:0] slice bug.
Eliminates the read/write collision symptoms in the register map."
```

---

### Task 5: Full-design elaboration + ldf integrity

**Files:** none modified (verification only)

- [ ] **Step 1: Elaborate the full synthesized source set for syntax/port regressions**

Run (lint/elaborate only — Diamond IP `PLL`/`efb_i2c` are black boxes, so expect "unknown module" notes for those, which are acceptable):
```
iverilog -g2012 -t null -s top src/top.v src/registers.v src/i2c_slave_top.v src/i2cslave_controller_top.v src/i2cslave_controller.v src/adc_control.v src/dds_gain_control.v src/dds_control_interface.v src/heart_beat.v src/reset_generator.v src/filter.v
```
Expected: no errors originating in `registers.v` and no port-mismatch errors at the `registers`/`i2c_slave_top` instantiation. (PLL/efb_i2c missing-module notes are fine.)

- [ ] **Step 2: Confirm the build source set is unchanged**

Run: `git diff --stat HEAD~3 -- seed_driver.ldf`
Expected: **no output** (the `.ldf` was not modified; `tb_registers.v` is intentionally not added to it).

- [ ] **Step 3: Re-run the testbench once more for a clean record**

Run the compile+run commands from Task 1 Step 2. Expected: `ERRORS=0`. Capture the full PASS output as the "after" record.

---

### Task 6: Final review

- [ ] **Step 1: Diff review**

Run: `git diff HEAD~4 -- src/registers.v`
Confirm: identical port list to the original; every field has exactly one driving `always` block; no `negedge clk`, no signal used as a clock (`posedge start/stop/data_vld*`), no shared `update_count`.

- [ ] **Step 2: Request code review**

Invoke `superpowers:requesting-code-review` on the branch diff. Address findings per `superpowers:receiving-code-review`.

- [ ] **Step 3: Hardware caveat note**

Record for the user: simulation confirms the functional fixes (round-trip, slice, lost-update, control). The metastability portion of the addressing/sync symptom is fixed by the async→sync restructuring and is **not** provable in zero-delay sim — it needs on-hardware confirmation after a Diamond build + bitstream.

---

## Self-review notes

- **Spec coverage:** symptom 1 (lost updates) → T2 + per-field one-shots (Task 4); symptom 2 (wrong byte) → T3 + slice fix + single read mux; symptom 3 (glitch/overwrite) → control single-writer block (Task 4) + T5; symptom 4 (addressing/sync) → synchronous pointer FSM + r_w sync (Task 4), functional part guarded by T1/T4, metastability part noted as hardware-only (Task 6 Step 3). Interface preserved → Task 5. Sim-only TB / ldf unchanged → Task 5 Step 2.
- **Mapping risk:** explicitly handled by Task 2 (measure) → Task 4 Step 1/Step 2 (match + iterate). The plan does not assume the idealized mapping is correct.
- **Non-goals honored:** stretch stays disabled (`stretch_on = 1'b0`); no Diamond P&R; address map and defaults preserved.
