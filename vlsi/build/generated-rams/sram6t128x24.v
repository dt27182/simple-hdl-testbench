`timescale 1ns/10ps

module sram6t128x24 ( A1, CE1, WEB1, WBM1, OEB1, CSB1, I1, O1 );
input CE1;
input WEB1;
input OEB1;
input CSB1;
input [6:0] A1;
input [23:0] I1;
output reg [23:0] O1;
input [2:0] WBM1;

reg notifier;

specify
$setuphold(posedge CE1, WEB1, 0, 0, notifier);
$setuphold(posedge CE1, OEB1, 0, 0, notifier);
$setuphold(posedge CE1, CSB1, 0, 0, notifier);
$setuphold(posedge CE1, A1[0], 0, 0, notifier);
$setuphold(posedge CE1, A1[1], 0, 0, notifier);
$setuphold(posedge CE1, A1[2], 0, 0, notifier);
$setuphold(posedge CE1, A1[3], 0, 0, notifier);
$setuphold(posedge CE1, A1[4], 0, 0, notifier);
$setuphold(posedge CE1, A1[5], 0, 0, notifier);
$setuphold(posedge CE1, A1[6], 0, 0, notifier);
$setuphold(posedge CE1, I1[0], 0, 0, notifier);
$setuphold(posedge CE1, I1[1], 0, 0, notifier);
$setuphold(posedge CE1, I1[2], 0, 0, notifier);
$setuphold(posedge CE1, I1[3], 0, 0, notifier);
$setuphold(posedge CE1, I1[4], 0, 0, notifier);
$setuphold(posedge CE1, I1[5], 0, 0, notifier);
$setuphold(posedge CE1, I1[6], 0, 0, notifier);
$setuphold(posedge CE1, I1[7], 0, 0, notifier);
$setuphold(posedge CE1, I1[8], 0, 0, notifier);
$setuphold(posedge CE1, I1[9], 0, 0, notifier);
$setuphold(posedge CE1, I1[10], 0, 0, notifier);
$setuphold(posedge CE1, I1[11], 0, 0, notifier);
$setuphold(posedge CE1, I1[12], 0, 0, notifier);
$setuphold(posedge CE1, I1[13], 0, 0, notifier);
$setuphold(posedge CE1, I1[14], 0, 0, notifier);
$setuphold(posedge CE1, I1[15], 0, 0, notifier);
$setuphold(posedge CE1, I1[16], 0, 0, notifier);
$setuphold(posedge CE1, I1[17], 0, 0, notifier);
$setuphold(posedge CE1, I1[18], 0, 0, notifier);
$setuphold(posedge CE1, I1[19], 0, 0, notifier);
$setuphold(posedge CE1, I1[20], 0, 0, notifier);
$setuphold(posedge CE1, I1[21], 0, 0, notifier);
$setuphold(posedge CE1, I1[22], 0, 0, notifier);
$setuphold(posedge CE1, I1[23], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[0], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[1], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[2], 0, 0, notifier);
(posedge CE1 => O1[0]) = (0.3:0.3:0.3);
(posedge CE1 => O1[1]) = (0.3:0.3:0.3);
(posedge CE1 => O1[2]) = (0.3:0.3:0.3);
(posedge CE1 => O1[3]) = (0.3:0.3:0.3);
(posedge CE1 => O1[4]) = (0.3:0.3:0.3);
(posedge CE1 => O1[5]) = (0.3:0.3:0.3);
(posedge CE1 => O1[6]) = (0.3:0.3:0.3);
(posedge CE1 => O1[7]) = (0.3:0.3:0.3);
(posedge CE1 => O1[8]) = (0.3:0.3:0.3);
(posedge CE1 => O1[9]) = (0.3:0.3:0.3);
(posedge CE1 => O1[10]) = (0.3:0.3:0.3);
(posedge CE1 => O1[11]) = (0.3:0.3:0.3);
(posedge CE1 => O1[12]) = (0.3:0.3:0.3);
(posedge CE1 => O1[13]) = (0.3:0.3:0.3);
(posedge CE1 => O1[14]) = (0.3:0.3:0.3);
(posedge CE1 => O1[15]) = (0.3:0.3:0.3);
(posedge CE1 => O1[16]) = (0.3:0.3:0.3);
(posedge CE1 => O1[17]) = (0.3:0.3:0.3);
(posedge CE1 => O1[18]) = (0.3:0.3:0.3);
(posedge CE1 => O1[19]) = (0.3:0.3:0.3);
(posedge CE1 => O1[20]) = (0.3:0.3:0.3);
(posedge CE1 => O1[21]) = (0.3:0.3:0.3);
(posedge CE1 => O1[22]) = (0.3:0.3:0.3);
(posedge CE1 => O1[23]) = (0.3:0.3:0.3);
endspecify

reg [23:0] memory[127:0];
always @ (posedge CE1)
begin
  if (~CSB1 & WEB1)
    O1 <= memory[A1];
  else if (~CSB1 & ~WEB1)
  begin
    if (WBM1[0])
      memory[A1][7:0] <= I1[7:0];
    if (WBM1[1])
      memory[A1][15:8] <= I1[15:8];
    if (WBM1[2])
      memory[A1][23:16] <= I1[23:16];
  end
end


endmodule
