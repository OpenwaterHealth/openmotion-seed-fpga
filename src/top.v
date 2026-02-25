///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: <Name>
//
// File: top.v
// File history:
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//
// Description: 
//
// <Description here>
//
// Targeted device: <Family::ProASIC3> <Die::A3PN010> <Package::48 QFN>
// Author: <Name>
//
/////////////////////////////////////////////////////////////////////////////////////////////////// 
`timescale 1ns / 1ps

module top( 
    input     rstn,                 	// Pin 34
    input     system_reset_n,         	// Pin 97

    input     clk_25mhz,              	// Pin 1
	
	input 	  scl_cfg,
	inout	  sda_cfg,
	output    seed_mod_mosi,        	// Pin 29    
	output    seed_mod_sck,         	// Pin 30
	output    seed_mod_ss,          	// Pin 32
	
	//output    seed_laser_disable,   	// Pin 17
	output    seed_laser_en_led_n,    	// Pin 74

	input     seed_adc_sdo1,        	// Pin 10
	input     seed_adc_sdo2,        	// Pin 14
	output    seed_adc_sck,         	// Pin 15
	output    seed_adc_convert,     	// Pin 16
	
    inout     scl,             		// Pin 18
    inout     sda,             		// Pin 19

	input     trigger,        	        // Pin 17//
	input     seed_compared,        	// Pin 28
	output    over_current_shutdown_n, // Pin 48
	
	output    seed_dac_mosi,        	// Pin 20
	output    seed_dac_ss,          	// Pin 21
	output    seed_dac_sck,         	// Pin 24
	output    seed_ldac_n,          	// Pin 27
	output    seed_reset_n,          	// Pin 25

    output    heartbeat_n,            	// Pin 51
	inout     mcu_gpio,             	// Pin 59
	
	inout     seed_spare1,          	// Pin 84
	inout     seed_spare2,          	// Pin 81
	inout     seed_spare3,          	// Pin 75
	inout     seed_spare4,          	// Pin 78
	
	inout     seed_gpio1,           	// Pin 69
	inout     seed_gpio2,           	// Pin 71
	inout     seed_gpio3,           	// Pin 70
	inout     seed_gpio4,            	// Pin 68
	
	output    spare_led1_n,          	// Pin 60
	output    spare_led2_n             	// Pin 61

);

wire        buf_rstn;
wire        buf_clk;
wire        buf_sclk;
wire        buf_miso;
wire        buf_laser_active;
wire        adc_data_valid;
wire [15:0] adc_voltage_data;
wire [15:0] adc_current_data;
wire [7:0]  status;

wire [15:0] modulate_delay;
wire [15:0] dds_gain;
wire [15:0] cw_gain;
wire [15:0] dds_current_limit;
wire [15:0] cw_current_limit;
wire [15:0] adc_current_limit;
wire [27:0] modulate_frequency;
wire [13:0] modulate_phrase;

wire modulate_configurate;
wire modulate_enable;

wire [15:0] static_control;
wire [15:0] control;
wire        over_current_limit;
wire [7:0]  revision;
wire [7:0]  minor;
wire [7:0]  major;
wire [7:0]  ID;

wire dds_mon_current_update;
wire cw_mon_current_update;

wire dds_gain_update;
wire cw_gain_update;
wire dds_current_limit_update;
wire cw_current_limit_update;
wire dds_mon_current_limit_update;
wire cw_mon_current_limit_update;

wire spi_dds_gain_control_ready,spi_dds_control_ready;
wire dds_gain_control_mosi,dds_control_mosi;
wire dds_gain_control_sck,dds_control_sck;
wire dds_cw_control_select;
wire dds_cw_mode_select;
wire buf_shutdown_n;

wire start;
wire stop;
wire data_vld;
wire r_w;
wire [7:0] data_in;
wire [7:0] data_out;
wire data_valid_dbg;
wire auto_run;
wire test_mode;
wire lock;
wire clkx2;

wire start_modulate,stop_modulate;
wire sop,eop;
wire test;

assign modulate_configurate     = control[0];
assign mcu_gpio                 = control[15];

assign modulate_enable          = static_control[0];
assign laser_active             = static_control[1];
assign auto_run                 = static_control[4];
assign test_mode                = static_control[7];

assign over_current_shutdown_n  = !(over_current_limit);
assign seed_laser_en_led_n  = !laser_active;
//assign seed_laser_disable = !(over_current_limit);   // disable= LOW

//assign seed_spare4              = dds_gain_update;
//assign seed_spare2              = seed_adc_sdo1;
//assign seed_spare2              = seed_adc_sdo2;
//assign seed_spare1              = seed_adc_sck;
//assign seed_spare2              = seed_adc_convert;
//assign seed_spare1              = seed_ldac_n;
//assign seed_spare1              = trigger;
//assign seed_spare2              = test;
//assign seed_spare2              = start_modulate;
//assign seed_spare3              = stop_modulate;
//assign seed_spare4              = sop;
//assign seed_spare4              = seed_mod_sck;


assign seed_spare1              = seed_dac_sck;
assign seed_spare2              = seed_dac_mosi;
assign seed_spare3              = seed_dac_ss;
assign seed_spare4              = seed_ldac_n;
assign seed_gpio1               = 0;
assign seed_gpio2               = 0;
assign seed_gpio3               = 0;
assign seed_gpio4               = 0;
assign spare_led1_n             = 0;
assign spare_led2_n             = 0;
		

assign status = {4'h0,system_reset_n,laser_active,seed_compared,over_current_limit};

assign buf_rstn = rstn  & system_reset_n;
assign seed_reset_n = 1;
assign revision = 8'h8;
assign minor    = 8'h0;
assign major    = 8'h0;
assign ID       = 8'h1;

reset_generator reset_generator( 
    .rstn      (rstn),
    .clk       (buf_clk),
    .lock      (lock),
    .reset_n   (reset_n)
);

PLL PLL( 
    .RST    (!rstn),
    .CLKI   (clk_25mhz),
    .CLKOP  (buf_clk),
    .CLKOS  (clkx2),
    .LOCK   (lock)
);

	
efb_i2c efb_inst (
	// Wishbone clock (MANDATORY)
	.wb_clk_i(clk_25mhz),
	.wb_rst_i(1'b0),

	// Wishbone interface (unused, but must exist)
	.wb_stb_i(1'b0),
	.wb_cyc_i(1'b0),
	.wb_we_i(1'b0),
	.wb_adr_i(8'b0),
	.wb_dat_i(8'b0),

	// Outputs (unused)
	.wb_ack_o(),
	.wb_dat_o(),
	.i2c1_irqo(),

	// I2C pins
	.i2c1_scl(scl_cfg),
	.i2c1_sda(sda_cfg)

	// SPI / Timer / UART ports can be left unconnected
);

heart_beat heart_beat( 
    .rstn      (reset_n),
    .clk       (buf_clk),
    .heartbeat (heartbeat_n)
);

i2c_slave_top i2c_slave_top (
	.rstn 							(reset_n),
	.clk 							(buf_clk),
	
	.scl 							(scl),
	.sda 							(sda),
	
    .adc_voltage_data 				(adc_voltage_data),
    .adc_current_data 				(adc_current_data),
    .monitor_status 				(monitor_status),
    .status 						(status),
	
	.revision 						(revision),
	.minor 				    		(minor),
	.major 				    		(major),
	.ID 				    		(ID),

    .dds_gain 						(dds_gain),
    .cw_gain 						(cw_gain),
    .dds_current_limit 				(dds_current_limit),
    .cw_current_limit 				(cw_current_limit),
    .modulate_frequency    			(modulate_frequency),
    .modulate_phrase    			(modulate_phrase),
    .dds_gain_update 		   		(dds_gain_update),
    .cw_gain_update 		   		(cw_gain_update),
    .dds_current_limit_update 		(dds_current_limit_update),
    .cw_current_limit_update  		(cw_current_limit_update),
    .dds_mon_current_limit_update 	(dds_mon_current_limit_update),
    .cw_mon_current_limit_update  	(cw_mon_current_limit_update),

    .dds_mon_current_limit 			(dds_mon_current_limit),
    .cw_mon_current_limit 			(cw_mon_current_limit),
    .control 	            		(control),
    .static_control 	    		(static_control)

);
 
dds_gain_control dds_gain_control(
    .rstn               		(reset_n),
    .clk                		(buf_clk),

    .dds_gain           		(dds_gain),
    .cw_gain            		(cw_gain),
    .dds_current_limit  		(dds_current_limit),
    .cw_current_limit   		(cw_current_limit),

    .dds_gain_update          	(dds_gain_update),
    .cw_gain_update           	(cw_gain_update),
    .dds_current_limit_update 	(dds_current_limit_update),
    .cw_current_limit_update  	(cw_current_limit_update),

    .mosi               		(seed_dac_mosi),
    .ss                 		(seed_dac_ss),
    .sck                		(seed_dac_sck),
    .ldac_n             		(seed_ldac_n)
	);


dds_control_interface dds_control_interface(
    .rstn               	(reset_n),
    .clk_d2                	(buf_clk),
    .clk                	(clkx2),

    .trigger        	  	(trigger),
    .modulate_configurate 	(modulate_configurate),
    .modulate_enable      	(modulate_enable),
    .modulate_frequency   	(modulate_frequency),
    .modulate_phrase    	(modulate_phrase),

    .test        			(test),
    .sop        			(sop),
    .eop			        (eop),
    .start_modulate        (start_modulate),
    .stop_modulate         (stop_modulate),
    .mosi               	(seed_mod_mosi),
    .ss0                	(seed_mod_ss),
    .sck                	(seed_mod_sck),
	.data_valid_dbg     	(data_valid_dbg)
	);


adc_control adc_control( 
    .rstn                   		(reset_n),
    .clk                    		(buf_clk),

    .adc_sdo1               		(seed_adc_sdo1),
    .adc_sdo2               		(seed_adc_sdo2),
    .dds_cw_mode_select     		(dds_cw_mode_select),
    .dds_mon_current_limit  		(dds_mon_current_limit),
    .cw_mon_current_limit   		(cw_mon_current_limit),
    .dds_mon_current_limit_update 	(dds_mon_current_limit_update),
    .cw_mon_current_limit_update  	(cw_mon_current_limit_update),
    .adc_status_clear       		(adc_status_clear),

    .adc_data_valid         		(adc_data_valid),
    .adc_voltage_data       		(adc_voltage_data),
    .adc_current_data       		(adc_current_data),
    .adc_sck                		(seed_adc_sck),
    .adc_convert            		(seed_adc_convert),

    .monitor_status         		(monitor_status)

);

endmodule

