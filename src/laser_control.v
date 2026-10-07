// SPDX-License-Identifier: AGPL-3.0-or-later
///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: <Name>
//
// File: laser_control.v
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

module laser_control(
	input     clk,
    input     rstn,

    input     modulate_on,
    input     cw_on,

	input     laser_active,         // Pin 12
    input     Analog_power_Good,    // Pin 47
    input     shutdown_n,           // Pin 37

	output reg  clear_n,              // Pin 13
    output reg  laser_disable_n,      // Pin 38
    output reg  modulate_active_n,    // Pin 40
    output reg  cw_active_n           // Pin 41
	);
		
	always @(posedge clk or negedge rstn) begin
		if (!rstn) begin
             laser_disable_n <= 1;
             modulate_active_n <= 1;
             cw_active_n <= 1;
             clear_n <= 1;
        end else begin
                    laser_disable_n <= !(laser_active & Analog_power_Good) &! shutdown_n;
                    modulate_active_n <= !(modulate_on & laser_active & Analog_power_Good) & !shutdown_n;
                    cw_active_n <= cw_on & !(laser_active & Analog_power_Good) & shutdown_n;
		        end
	end

endmodule
