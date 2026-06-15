`timescale 1ns / 1ps
// Sim-only testbench for the I2C register map. NOT part of seed_driver.ldf.
//
// GOLDEN MAPPING (measured 2026-06-15 against the current registers.v):
//   - A write transaction's first byte after the slave address is the register
//     INDEX (pointer). The first DATA byte lands at addr=INDEX, the next at
//     INDEX+1, etc. No off-by-one.
//   - A read transaction starts at the last-written INDEX and auto-increments.
//   Verified: write index=0x02 data={0x11,0x22,0x33} -> reg[0x02]=0x11,
//   reg[0x03]=0x22, reg[0x04]=0x33; dds_gain(0x02/0x03 lo/hi)=0x2211.
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

    task check8(input [255:0] name, input [7:0] got, input [7:0] exp);
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL %0s: got %02h exp %02h", name, got, exp);
            end else $display("PASS %0s = %02h", name, got);
        end
    endtask

    task check16(input [255:0] name, input [15:0] got, input [15:0] exp);
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
        // The slave controller's FSMs reset on posedge i_rst (= posedge !rstn),
        // so rstn must go high->low->high to generate that edge (real hardware
        // gets it from power-on). Starting at 0 and only releasing never resets it.
        rstn = 1'b1;
        repeat (4) @(posedge clk);
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

        // Characterization: prove first-data-byte->index mapping + auto-increment.
        // Write index 0x02 with three bytes, then read 0x02..0x04 back.
        wbuf = '{8'h11, 8'h22, 8'h33};
        MASTER.i2c_write(SLAVE, 8'h02, 3, wbuf);
        repeat (50) @(posedge clk);
        check16("char_dds_gain", dds_gain, 16'h2211);   // lo=0x02=0x11, hi=0x03=0x22
        rbuf = '{8'h00, 8'h00, 8'h00};
        MASTER.i2c_read(SLAVE, 8'h02, 3, rbuf);
        check8("char_rd_0x02", rbuf[0], 8'h11);
        check8("char_rd_0x03", rbuf[1], 8'h22);
        check8("char_rd_0x04", rbuf[2], 8'h33);

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
        wbuf = '{8'h78, 8'h56, 8'h34, 8'h02};    // 0xA..0xD -> 0x0234_5678 (28-bit)
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

        $display("ERRORS=%0d", errors);
        $finish;
    end

endmodule
