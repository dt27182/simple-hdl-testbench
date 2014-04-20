`timescale 1ns/10ps

module sram6t128x48 ( A1, CE1, WEB1, WBM1, OEB1, CSB1, I1, O1 );
input CE1;
input WEB1;
input OEB1;
input CSB1;
input [6:0] A1;
input [47:0] I1;
output reg [47:0] O1;
input [5:0] WBM1;

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
$setuphold(posedge CE1, I1[24], 0, 0, notifier);
$setuphold(posedge CE1, I1[25], 0, 0, notifier);
$setuphold(posedge CE1, I1[26], 0, 0, notifier);
$setuphold(posedge CE1, I1[27], 0, 0, notifier);
$setuphold(posedge CE1, I1[28], 0, 0, notifier);
$setuphold(posedge CE1, I1[29], 0, 0, notifier);
$setuphold(posedge CE1, I1[30], 0, 0, notifier);
$setuphold(posedge CE1, I1[31], 0, 0, notifier);
$setuphold(posedge CE1, I1[32], 0, 0, notifier);
$setuphold(posedge CE1, I1[33], 0, 0, notifier);
$setuphold(posedge CE1, I1[34], 0, 0, notifier);
$setuphold(posedge CE1, I1[35], 0, 0, notifier);
$setuphold(posedge CE1, I1[36], 0, 0, notifier);
$setuphold(posedge CE1, I1[37], 0, 0, notifier);
$setuphold(posedge CE1, I1[38], 0, 0, notifier);
$setuphold(posedge CE1, I1[39], 0, 0, notifier);
$setuphold(posedge CE1, I1[40], 0, 0, notifier);
$setuphold(posedge CE1, I1[41], 0, 0, notifier);
$setuphold(posedge CE1, I1[42], 0, 0, notifier);
$setuphold(posedge CE1, I1[43], 0, 0, notifier);
$setuphold(posedge CE1, I1[44], 0, 0, notifier);
$setuphold(posedge CE1, I1[45], 0, 0, notifier);
$setuphold(posedge CE1, I1[46], 0, 0, notifier);
$setuphold(posedge CE1, I1[47], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[0], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[1], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[2], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[3], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[4], 0, 0, notifier);
$setuphold(posedge CE1, WBM1[5], 0, 0, notifier);
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
(posedge CE1 => O1[24]) = (0.3:0.3:0.3);
(posedge CE1 => O1[25]) = (0.3:0.3:0.3);
(posedge CE1 => O1[26]) = (0.3:0.3:0.3);
(posedge CE1 => O1[27]) = (0.3:0.3:0.3);
(posedge CE1 => O1[28]) = (0.3:0.3:0.3);
(posedge CE1 => O1[29]) = (0.3:0.3:0.3);
(posedge CE1 => O1[30]) = (0.3:0.3:0.3);
(posedge CE1 => O1[31]) = (0.3:0.3:0.3);
(posedge CE1 => O1[32]) = (0.3:0.3:0.3);
(posedge CE1 => O1[33]) = (0.3:0.3:0.3);
(posedge CE1 => O1[34]) = (0.3:0.3:0.3);
(posedge CE1 => O1[35]) = (0.3:0.3:0.3);
(posedge CE1 => O1[36]) = (0.3:0.3:0.3);
(posedge CE1 => O1[37]) = (0.3:0.3:0.3);
(posedge CE1 => O1[38]) = (0.3:0.3:0.3);
(posedge CE1 => O1[39]) = (0.3:0.3:0.3);
(posedge CE1 => O1[40]) = (0.3:0.3:0.3);
(posedge CE1 => O1[41]) = (0.3:0.3:0.3);
(posedge CE1 => O1[42]) = (0.3:0.3:0.3);
(posedge CE1 => O1[43]) = (0.3:0.3:0.3);
(posedge CE1 => O1[44]) = (0.3:0.3:0.3);
(posedge CE1 => O1[45]) = (0.3:0.3:0.3);
(posedge CE1 => O1[46]) = (0.3:0.3:0.3);
(posedge CE1 => O1[47]) = (0.3:0.3:0.3);
endspecify

reg [47:0] memory[127:0];
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
    if (WBM1[3])
      memory[A1][31:24] <= I1[31:24];
    if (WBM1[4])
      memory[A1][39:32] <= I1[39:32];
    if (WBM1[5])
      memory[A1][47:40] <= I1[47:40];
  end
end


endmodule
