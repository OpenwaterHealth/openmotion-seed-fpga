`timescale 1ns / 1ps

module dds_gain_control(
	input        clk,
    input        rstn,
 
    input [15:0] dds_gain,
    input [15:0] cw_gain,

    input [15:0] dds_current_limit,    //16'h1072;    // Limit 80mA
    input [15:0] cw_current_limit,     //16'h20e5;    // limit 160mA

    input        dds_gain_update,
    input        cw_gain_update,
    input        dds_current_limit_update,
    input        cw_current_limit_update,

    output       mosi,
    output       ss,
    output       sck,
    output       spi_ready,
    output       ldac_n
	);
	
localparam IDLE      = 0;
localparam WAIT      = 1;
localparam SEND      = 2;
localparam SCK_STATE = 3;
localparam READY     = 4;
localparam LDAC      = 5;
localparam DONE      = 6;
		
localparam WRITE_UPDATE_DAC_CHANNEL = 4'h3;
localparam DDS_DAC = 4'h1;
localparam CW_DAC  = 4'h8;

reg [3:0]  state=0,cstate=0,dac_state=0;
reg [7:0]  count;
reg [23:0] data_temp;
reg [23:0] data;
reg        data_valid;
reg        cw_data_valid;
reg        data_ready;
reg        dds_gain_select;
reg        cw_select;
reg        data_valid_reset;

reg        ss1_temp=1;
reg        sck_temp=0;
reg        mosi_temp=0;
reg        ldac_temp=1;

reg [15:0] dds_gain_reg,dds_gain_d1;
reg [15:0] cw_gain_reg,cw_gain_d1;
reg [15:0] dds_current_limit_reg;
reg [15:0] cw_current_limit_reg;
reg dds_gain_update_d1,cw_gain_update_d1;
reg [15:0] dds_gain_reg_old;
reg [15:0] cw_gain_reg_old;

assign ss = ss1_temp;
assign sck = sck_temp;
assign mosi = mosi_temp;
assign ldac_n = ldac_temp;
assign spi_ready = data_ready;

reg [15:0] total_count,dac_count;
reg mod_data_select,cw_data_select;
reg data_ready_d;
reg ss1_temp_d;

always @(posedge clk or negedge rstn) begin
    begin
          if (!rstn) begin
               dds_gain_reg <= 16'h0;
            //   cw_gain_reg <= 16'h0839;    // Revision 0.1
               cw_gain_reg <= 16'h07f5;    // Revision 0.2
               dds_gain_d1 <= 16'h0;
               cw_gain_d1 <= 16'h07f5;
               dds_gain_update_d1 <= 0;
               cw_gain_update_d1 <= 0;
          end else begin
                         dds_gain_d1 <= dds_gain;
                         cw_gain_d1 <= cw_gain;
                         dds_gain_update_d1 <= dds_gain_update;
                         cw_gain_update_d1 <= cw_gain_update;
                         if (dds_gain_update_d1) begin
                             if (dds_gain_d1 < dds_current_limit_reg) begin
                                 dds_gain_reg <= dds_gain_d1;
                             end
                         end
                         if (cw_gain_update_d1) begin
                             if (cw_gain_d1 < cw_current_limit_reg) begin
                                 cw_gain_reg <= cw_gain_d1;
                             end
                         end
                    end
        end
end


always @(posedge clk or negedge rstn) begin
		if (!rstn) begin
             total_count <= 0;
             mod_data_select <= 0;
             cw_data_select <= 0;
             ss1_temp <= 1;
             sck_temp <= 0;
             ss1_temp_d <= 0;
             data_ready <= 1;
			 data_valid_reset <= 0;

             cstate <= IDLE;
        end else begin
                      ss1_temp_d <= ss1_temp;

                      case (cstate)
                            IDLE : begin
                                        total_count <= 0;
										data_valid_reset <= 0;

                                        if (data_valid) begin
                                           ss1_temp <= 0; 
                                           data_ready <= 0;
                                           cstate <= SCK_STATE;
                                        end
                                   end
                       SCK_STATE : begin
                                      if (total_count > 47) begin
                                          sck_temp <= 0;
                                          cstate <= DONE;
                                      end else begin
                                                    sck_temp <= ~sck_temp;
                                                    total_count <= total_count + 1;
                                               end
                                    end
                        DONE: begin
							     data_valid_reset <= 1;
                                 mod_data_select <= 0;
                                 cw_data_select <= 0;
                                 total_count <= 0;
                                 ss1_temp <= 1; 
                                 data_ready <= 1;
                                 cstate <= IDLE;
                                 sck_temp <= 0;
                              end
                    endcase
		end
	end

always @(posedge sck_temp or negedge rstn or posedge data_valid_reset) begin
    if (!rstn | data_valid_reset) begin
             count <= 22;
             mosi_temp <= 0;
             data_temp <= 0;
             state <= IDLE;
        end else begin
                     case(state)
                          IDLE: begin
                                   if (!ss1_temp) begin
                                        mosi_temp <= data[23];
                                        data_temp <= data << 1;
                                        state <= SEND;
                                   end
                                end
                          SEND: begin
                                   mosi_temp <= data_temp[23];
                                   data_temp <= data_temp << 1;
                                   if (count == 0) begin
                                       count <= 22;
                                       state <= DONE;
                                   end else count <= count - 1;
                                end
                          DONE: begin
							         data_temp <= 0;
                                     mosi_temp <= 0;
                                     state <= IDLE;
                                end
                     endcase
		        end
end

always @(posedge clk or negedge rstn) begin
	if (!rstn) begin
         ldac_temp <= 1;
         data_ready_d <= 0;
         dac_count <= 0;
         dac_state <= IDLE;
    end else begin
                 data_ready_d <= data_ready;
                 case (dac_state)
	                    IDLE : if (data_valid) dac_state <= READY;
	                   READY : if (!data_ready_d & data_ready) dac_state <= LDAC;
	                    LDAC : begin
                                    if (dac_count > 10) begin
                                        ldac_temp <= 0; 
                                        dac_count <= 0;
                                        dac_state <= DONE;
                                    end else begin
                                                dac_count <= dac_count + 1;
                                                ldac_temp <= 1;
                                             end
                               end
	                    DONE : begin
                                    if (dac_count > 4) begin
                                        ldac_temp <= 1; 
                                        dac_count <= 0; 
                                        dac_state <= IDLE;
                                    end else dac_count <= dac_count + 1;
                               end
                 endcase
		     end
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        dds_current_limit_reg <= 16'h03dc;    // Limit 80mA Version 0.2
        cw_current_limit_reg <= 16'h08c0;     // limit 160mA Version 0.2
       // dds_current_limit_reg <= 16'h08d0;    // Limit 80mA Version 0.1
       // cw_current_limit_reg <= 16'h08d0;     // limit 160mA Version 0.1
    end else begin
                 if (dds_current_limit_update) dds_current_limit_reg <= dds_current_limit;
                 if (cw_current_limit_update) cw_current_limit_reg <= cw_current_limit;
             end
end


always @(posedge clk or negedge rstn or posedge data_valid_reset) begin
    if (!rstn | data_valid_reset) begin
         data <= 0;
         data_valid <= 0;
    end else begin					 
                 if (dds_gain_update_d1) begin
                     data <= {WRITE_UPDATE_DAC_CHANNEL,4'h1,dds_gain_reg};
					 data_valid <= 1;
				 end 
				 
				 if (cw_gain_update_d1) begin
					 data <= {WRITE_UPDATE_DAC_CHANNEL,4'h8,cw_gain_reg};
					 data_valid <= 1;
				 end
             end
end


endmodule