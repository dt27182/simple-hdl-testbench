`ifndef VCS
`ifndef DC
`define BEHAVIORAL
`endif
`endif

module PartialProductMultiplier #(parameter W0 = 1, parameter W1 = 1)
(
  input [W0-1:0] io_in0,
  input [W1-1:0] io_in1,
  output [W0+W1+1:0] io_out0,
  output [W0+W1+1:0] io_out1
);

`ifdef BEHAVIORAL
  assign io_out0 = $signed(io_in0) * $signed(io_in1);
  assign io_out1 = 1'b0;
`else
  DW02_multp #(.a_width(W0), .b_width(W1), .out_width(W0+W1+2), .verif_en(3)) multp
  (
    .a(io_in0),
    .b(io_in1),
    .tc(1'b1),
    .out0(io_out0),
    .out1(io_out1)
  );
`endif

endmodule
