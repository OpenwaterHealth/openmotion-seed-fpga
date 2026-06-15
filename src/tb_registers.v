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
