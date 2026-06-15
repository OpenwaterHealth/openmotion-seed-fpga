`timescale 1ns / 100ps

module registers(
    input               clk,
    input               rstn,
    input               SCL,            // unused: kept for interface compatibility
    output [7:0]        data_to_i2c,
    input               start,
    input               stop,           // unused: kept for interface compatibility
    input               data_vld,
    input               r_w,
    input [7:0]         i2c_to_data,
    output              stretch_on,
    input [15:0]        adc_current_data,
    input [15:0]        adc_voltage_data,
    input [7:0]         monitor_status,  // unused: kept for interface compatibility
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
    // Output mux selected by the raw (un-synchronized) r_w on purpose: this is a
    // combinational read-path select only, and the I2C engine re-latches i_data
    // in its own domain. Control/FSM logic below uses the 2-FF-synced r_w_sync.
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
                    // First read byte after (repeated) start serves start_addr;
                    // only advance the pointer for subsequent bytes.
                    if (awaiting_index)
                        awaiting_index <= 1'b0;
                    else
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
