`timescale 1ns / 1ps

module dds_control_interface(
	input        clk_d2,
	input        clk,
    input        rstn,
 
    input        trigger,
    input        modulate_configurate,
    input        modulate_enable,
    input [27:0] modulate_frequency,
    input [13:0] modulate_phrase,

    output reg   start_modulate,
    output reg   stop_modulate,
    output reg   test,
    output reg   sop,
    output reg   eop,
    output       mosi,
    output reg   ss0,
    output reg   sck,
    output       data_valid_dbg

	);
	
localparam IDLE       = 0;
localparam WAIT       = 1;
localparam SEND       = 2;
localparam CONFIG_MOD = 3;
localparam START      = 4;
localparam STOP       = 5;
localparam TRANSFER   = 6;
localparam SCK_STATE  = 7;
localparam DELAY      = 8;
localparam DONE       = 9;
		
reg [3:0]  state=0,cstate=0,config_state=0;
reg [7:0]  count,wait_count;
reg [15:0] data_temp;
reg [15:0] data;
reg        data_valid;
reg        cw_data_valid;
reg        data_valid_reset;
reg [15:0] delay_count;

reg        ss0_temp=1;
reg        ss0_temp2=1;
reg        sck_temp=0;
reg        sck_temp2= 0;
reg        mosi_temp=0;
reg [15:0] total_count,dac_count;
reg        mod_data_select,cw_data_select;
reg        mosi_reset;
reg        ss0_temp_d,ss0_temp_dd,ss0_temp_dd2;
reg [15:0] dds_control_reg;
reg         modulate_enable_d,modulate_enable_d2;
reg         trigger_d,trigger_d2;
reg         sck_d,sck_d2,sck_d3;

reg [15:0] dds_control_reg_old;
reg         transfer_completed,transfer_completed_temp,transfer_completed_temp_d,transfer_completed_temp_d2;
reg [3:0]  index=0;

reg transfer_done=0;
reg transfer_completed_reset,test_run;

assign mosi = mosi_temp;
assign data_valid_dbg = data_valid_reset;


always @(posedge clk or negedge rstn) begin
		if (!rstn) begin
             trigger_d <= 0;
             trigger_d2 <= 0;
             modulate_enable_d <= 0;
             modulate_enable_d2 <= 0;
        end else begin
                      trigger_d <= trigger;
                      trigger_d2 <= trigger_d;
					  modulate_enable_d <= modulate_enable;
		              modulate_enable_d2 <= modulate_enable_d;
                 end
end

always @(posedge clk_d2 or negedge rstn) begin
		if (!rstn) begin
             sck_d2 <= 1;
        end else begin
                      sck_d2 <= sck_d | ss0_temp;
                 end
end


always @(negedge clk_d2 or negedge rstn) begin
		if (!rstn) begin
             sck_d3 <= 1;
        end else begin
                      sck_d3 <= sck_d2;
                 end
end

always @(posedge clk_d2 or negedge rstn) begin
		if (!rstn) begin
             sck <= 1;
        end else begin
                      sck <= sck_d3;
                 end
end


always @(negedge clk or negedge rstn) begin
		if (!rstn) begin
             ss0_temp_dd <= 1;
             ss0_temp_dd2 <= 1;
        end else begin
                      ss0_temp_dd <= ss0_temp;
                      ss0_temp_dd2 <= ss0_temp_dd;
                 end
end


always @(negedge clk or negedge rstn) begin
		if (!rstn) begin
             ss0 <= 1;
             sck_temp2 <= 0;
        end else begin
                      ss0 <= ss0_temp2;
                      sck_temp2 <= sck_temp;
                 end
end

always @(posedge clk or negedge rstn) begin
		if (!rstn) begin
             ss0_temp2 <= 1;
             sck_d <= 0;
        end else begin
                      ss0_temp2 <= ss0_temp_dd | ss0_temp;
                      sck_d <= !sck_temp2;
                 end
end

always @(posedge clk or negedge rstn) begin
		if (!rstn) begin
             total_count <= 0;
             mod_data_select <= 0;
             cw_data_select <= 0;
             ss0_temp <= 1;
             sck_temp <= 0;
             ss0_temp_d <= 0;
             mosi_reset <= 0;
			 transfer_completed_temp <= 0;
			 transfer_completed_temp_d <= 0;
			 transfer_completed_temp_d2 <= 0;

             data_valid_reset <= 0;
             cstate <= IDLE;
        end else begin
					  transfer_completed_temp_d <= transfer_completed_temp;
					  transfer_completed_temp_d2 <= transfer_completed_temp_d;
                      ss0_temp_d <= ss0_temp;
                      if (!ss0_temp_d & ss0_temp) mosi_reset <= 1;
                      else mosi_reset <= 0;

                      case (cstate)
                            IDLE : begin
								        transfer_completed_temp <= 0;

                                        data_valid_reset <= 0;
                                        total_count <= 0;
                                        if (data_valid) begin
                                           cstate <= SCK_STATE;
                                        end
                                   end
                       SCK_STATE : begin
                                      ss0_temp <= 0; 
                                      if (total_count > 30) begin
                                          sck_temp <= 0;
                                          cstate <= WAIT;
                                      end else begin
                                                    sck_temp <= ~sck_temp;
                                                    total_count <= total_count + 1;
                                               end
                                    end
                        WAIT: begin
								 data_valid_reset <= 1;
                                 mod_data_select <= 0;
                                 cw_data_select <= 0;
                                 total_count <= 0;
                                 ss0_temp <= 1; 
                                 cstate <= DONE;
                                 sck_temp <= 0;
                              end
                        DONE: begin
								 transfer_completed_temp <= 1;
                                 cstate <= IDLE;
                              end
                    endcase
		end
	end

always @(negedge sck_temp or negedge rstn or posedge mosi_reset or posedge transfer_completed_reset) begin
		if (!rstn | mosi_reset | transfer_completed_reset) begin
             count <= 15;
             mosi_temp <= 0;
             data_temp <= 0;
             state <= IDLE;
        end else begin
                      case(state)
                              IDLE: begin
                                       if (!ss0_temp) begin
                                            count <= count - 1;
                                            mosi_temp <= data[15];
                                            data_temp <= data << 1;
                                            state <= SEND;
                                       end
                                    end
                              SEND: begin
                                       mosi_temp <= data_temp[15];
                                       data_temp <= data_temp << 1;
                                       if (count == 8'h0) begin
                                           count <= 14;
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
         transfer_completed_reset <= 0;
         modulate_enable_d <= 0;
         sop <= 0;
         eop <= 0;
         data <= 0;
         data_valid <= 0;
         start_modulate <= 0;
         stop_modulate <= 0;
         delay_count <= 0;
         index <= 0;
         test <= 0;
		 config_state <= IDLE;
    end else begin
		         modulate_enable_d <= modulate_enable;
				 case (config_state)
					  IDLE : begin
						        start_modulate <= 0;
						        stop_modulate <= 0;
						        data_valid <= 0;
						        if (modulate_configurate) config_state <= CONFIG_MOD;
						        if (!trigger_d2 & trigger_d) begin
									sop <= 1;
									config_state <= DELAY;
								end
						        if (!trigger_d & trigger_d2) begin
									eop <= 1;
									config_state <= DELAY;
								end
						     end
				     DELAY : begin
						        if (delay_count > 16'h0055) begin
									delay_count <= 0;
									if (sop) begin
										sop <= 0;
										config_state <= START;
								    end
									if (eop) begin
										eop <= 0;
										config_state <= STOP;
								    end
						        end else delay_count <= delay_count + 1;
						     end
				     START : begin
								if (modulate_enable_d2) begin
									data <= 16'h2002;
									data_valid <= 1;
									start_modulate <= 1;
									config_state <= IDLE;
								end else begin
												config_state <= IDLE;
												data_valid <= 0;
										 end
						     end
				      STOP : begin
						        test <= trigger;
									data <= 16'h2100;
									data_valid <= 1;
									stop_modulate <= 1;
									config_state <= IDLE;
						     end
				  CONFIG_MOD : begin //2
						        test <= 0;
						        transfer_completed_reset <= 0;
								 if (index <5) begin
									 data_valid <= 1;
									 case (index)
									    0 : data <= 16'h2100;
								        1 : data <= {2'h1,modulate_frequency[13:0]};		// Frequency (LSB)
								        2 : data <= {2'h1,modulate_frequency[27:14]};		// Frequency (MSB)
								        3 : data <= {2'h3,modulate_phrase[13:0]};		    // phrase
								        4 : data <= 16'h2002;    // Triangle
								  default : config_state <= IDLE;
								 endcase
								    index <= index + 1;
								    config_state <= TRANSFER;
								end else begin
									            index <= 0;
												config_state <= IDLE;
										 end
							 end 

			     TRANSFER : begin //3
						        test <= 0;
					             data_valid <= 0;
								 if (transfer_completed_temp) begin
									 transfer_completed_reset <= 1;
									 config_state <= CONFIG_MOD;
							     end
							 end 
				     DONE : begin //6
						        test <= 0;
							    config_state <= IDLE;
						    end 
				endcase
             end
end


endmodule