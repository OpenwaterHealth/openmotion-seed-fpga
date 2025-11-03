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

//`timescale <time_units> / <precision>

module reset_generator( 
    input  rstn,
    input  clk,
    input  lock,

    output reset_n
);

//reg [3:0] count;
//always @(posedge clk,negedge rstn)
//begin
//    if (!rstn) begin
 //       reset_n <= 0;     
 //   end else begin
 //                if (count > 10) reset_n <= 1;
 //                else count <= count + 1;
 //           end
//end

  // internal signals
   reg  in_d1;
   reg  in_d2;
   reg  in_d3;
   reg  out_n;
   reg clk_d2;
      
   always @(posedge clk or negedge rstn) begin
      if(!rstn) begin
         clk_d2 <= 1'b0;
      end else begin
         clk_d2 <= !clk_d2;       
      end
   end


   always @(posedge clk_d2 or negedge rstn) begin
      if(!rstn) begin
         in_d1 <= 1'b0;
         in_d2 <= 1'b0;
         in_d3 <= 1'b0;
      end else begin
         in_d1 <= rstn;       // Synchronizer #1
         in_d2 <= in_d1;       // Synchronizer #2
         in_d3 <= in_d2;       // Filter
      end
   end

   // Filter - two consecutive samples must match
   //        - all glitches shorter than clock period are suppressed
   assign reset_n = out_n;
   always @(posedge clk_d2 or negedge rstn) begin
      if(!rstn) begin
         out_n <= 1'b0;
      end else if (in_d2 == in_d3) begin
         out_n <= in_d3;
      end
   end
   
endmodule

