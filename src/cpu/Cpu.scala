package Cpu
import Chisel._
import Common._
 
class Cpu extends Module {
  val io = new Bundle {
    var readData_0 = new DecoupledIO(Bits(width = 32))
    var readAddr_0 = new DecoupledIO(Bits(width = 4)).flip
    var imemPort_0 = new VarLatIO(4, 32)
    var dmemPort_0 = new VarLatIO(43, 32)
  }
  val pcReg_0 = Bits()
  val W0 = Bits()
  val AM_regWData_pcReg_0_thread_0_writeNum_0 = Bits()
  val W1 = Bool()
  val inst = Bits()
  val rs1 = Bits()
  val rs2 = Bits()
  val AM_memWAddr_regfile_0_thread_0_writeNum_2 = Bits()
  val op = Bits()
  val imm = Bits()
  val AM_regWData_pcReg_0_thread_0_writeNum_1 = Bits()
  val W2 = Bits()
  val isJmp = Bool()
  val W3 = Bits()
  val isNotJmp = Bool()
  val rs1Data = Bits()
  val rs2Data = Bits()
  val W4 = Bool()
  val W5 = Bool()
  val W6 = Bool()
  val W7 = Bits()
  val isExternalRead = Bool()
  val W8 = Bits()
  val W9 = Bool()
  val W10 = Bits()
  val operand1 = Bits()
  val W11 = Bits()
  val W12 = Bool()
  val operand2 = Bits()
  val AM_memWData_regfile_0_thread_0_writeNum_0 = Bits()
  val AM_memWData_regfile_0_thread_0_writeNum_1 = Bits()
  val W13 = Bits()
  val W14 = Bool()
  val W15 = Bits()
  val W16 = Bool()
  val W17 = Bool()
  val W18 = Bits()
  val W19 = Bool()
  val adderSel = Bool()
  val W20 = Bits()
  val W21 = Bool()
  val W22 = Bits()
  val W23 = Bool()
  val subtractSel = Bool()
  val W24 = Bits()
  val isLoad = Bool()
  val W25 = Bits()
  val isStore = Bool()
  val W26 = Bits()
  val W27 = Bits()
  val memWrite = Bits()
  val W28 = Bits()
  val W29 = Bits()
  val W30 = Bool()
  val AM_stage_valid_0 = Bool()
  val W31 = Bool()
  val AM_stage_stall_0 = Bool()
  val AM_stage_kill_0 = Bool()
  val AM_stage_NoRAW_0 = Bool()
  val AM_stage_NoIOBusy_0 = Bool()
  val AM_stage_thread_sel_id_0 = Bits(width=1)
  val AM_global_stall = Bool()
  val W32 = Bool()
  val W33 = Bits()
  val W34 = Bool()
  val W35 = Bits()
  val W36 = Bool()
  val AM_inputIO_bits_mux_readAddr_0 = Bits()
  val W37 = Bits()
  val W38 = Bool()
  val AM_varLatIO_resp_bits_mux_imemPort_0 = Bits()
  val W39 = Bits()
  val W40 = Bool()
  val AM_memWData_regfile_0_thread_0_writeNum_2 = Bits()
  val W41 = Bits()
  val W42 = Bool()
  val AM_reg_mux_pcReg_0 = Bits()
  val W43 = Bits()
  val W44 = Bool()
  val AM_mem_read_data_mux_regfile_0_read_port_num_0 = Bits()
  val W45 = Bits()
  val W46 = Bool()
  val AM_mem_read_data_mux_regfile_0_read_port_num_1 = Bits()
  val W47 = Bits()
  val W48 = Bool()
  val AM_mem_read_data_mux_regfile_0_read_port_num_2 = Bits()
  val AM_thread_sel_counter = Bits()
  val W49 = Bits()
  val W50 = Bits()
  val W51 = Bool()
  val W52 = Bits()
  val W53 = Bool()
  val W54 = Bits()
  val W55 = Bits()
  val AM_perStageThreadSel_stage_0_thread_0 = Bool()
  val W56 = Bool()
  val W57 = Bool()
  val AM_iobusy_readData_0 = Bool()
  val W58 = Bool()
  val W59 = Bool()
  val W60 = Bool()
  val AM_iobusy_readAddr_0 = Bool()
  val W61 = Bool()
  val AM_varLatIO_busy_imemPort_0 = Bool()
  val AM_varLatIO_busy_dmemPort_0 = Bool()
  val W62 = Bool()
  val W63 = Bool()
  val W64 = Bool()
  val W65 = Bool()
  val W66 = Bool()
  val W67 = Bool()
  val W68 = Bool()
  val W69 = Bool()
  val W70 = Bool()
  val W71 = Bool()
  val W72 = Bool()
  val W73 = Bool()
  val W74 = Bool()
  val W75 = Bool()
  val W76 = Bool()
  val W77 = Bool()
  val W78 = Bool()
  val W79 = Bool()
  val W80 = Bool()
  val W81 = Bool()
  val W82 = Bool()
  val W83 = Bool()
  val W84 = Bool()
  val W85 = Bool()
  val W86 = Bool()
  val W87 = Bool()
  val W88 = Bool()
  val W89 = Bool()
  val W90 = Bool()
  val W91 = Bool()
  val W92 = Bool()
  val W93 = Bool()
  val W94 = Bool()
  val W95 = Bool()
  val W96 = Bool()
  val W97 = Bool()
  val W98 = Bool()
  val W99 = Bool()
  val W100 = Bool()
  val W101 = Bool()
  val W102 = Bool()
  val W103 = Bool()
  val W104 = Bool()
  val W105 = Bool()
  val W106 = Bool()
  val W107 = Bool()
  val W108 = Bool()
  val W109 = Bool()
  val W110 = Bool()
  val W111 = Bool()
  val W112 = Bool()
  val W113 = Bool()
  val W114 = Bool()
  val W115 = Bool()
  val W116 = Bool()
  val W117 = Bool()
  val W118 = Bool()
  val W119 = Bool()
  val W120 = Bool()
  val W121 = Bool()
  val W122 = Bool()
  val W123 = Bool()
  val W124 = Bool()
  val W125 = Bool()
  val W126 = Bool()
  val W127 = Bool()
  val W128 = Bool()
  val W129 = Bool()
  val W130 = Bool()
  val W131 = Bool()
  val W132 = Bool()
  val W133 = Bool()
  val W134 = Bool()
  val W135 = Bool()
  val W136 = Bool()
  val W137 = Bool()
  val W138 = Bool()
  val W139 = Bool()
  val W140 = Bool()
  val W141 = Bool()
  val W142 = Bool()
  val W143 = Bool()
  val W144 = Bool()
  val W145 = Bool()
  val W146 = Bool()
  val W147 = Bool()
  val W148 = Bool()
  val W149 = Bool()
  val W150 = Bool()
  val W151 = Bool()
  val W152 = Bool()
  val W153 = Bool()
  val W154 = Bool()
  val W155 = Bool()
  val W156 = Bool()
  val AM_regWEn_pcReg_0_thread_0_writeNum_0 = Bool()
  val AM_regWEn_pcReg_0_thread_0_writeNum_1 = Bool()
  val AM_memWEn_regfile_0_thread_0_writeNum_0 = Bool()
  val AM_memWEn_regfile_0_thread_0_writeNum_1 = Bool()
  val AM_memWEn_regfile_0_thread_0_writeNum_2 = Bool()
  val W157 = Bool()
  val W158 = Bool()
  val W159 = Bool()
  val W160 = Bool()
  val W161 = Bool()
  val W162 = Bool()
  val W163 = Bool()
  val W164 = Bool()
  W0 := Bits(1, width = 4)
  AM_regWData_pcReg_0_thread_0_writeNum_0 := AM_reg_mux_pcReg_0 + W0
  W1 := Bool(true)
  W88 := W1
  io.imemPort_0.reqBits := AM_reg_mux_pcReg_0
  inst := AM_varLatIO_resp_bits_mux_imemPort_0
  rs1 := inst(11, 8)
  rs2 := inst(7, 4)
  AM_memWAddr_regfile_0_thread_0_writeNum_2 := inst(3, 0)
  op := inst(15, 12)
  imm := inst(31, 16)
  AM_regWData_pcReg_0_thread_0_writeNum_1 := imm(3, 0)
  W2 := Bits(6, width = 4)
  isJmp := op === W2
  W3 := Bits(6, width = 4)
  isNotJmp := op != W3
  W4 := Bool(true)
  W5 := Bool(true)
  W6 := Bool(true)
  W7 := Bits(7, width = 4)
  isExternalRead := op === W7
  W34 := isExternalRead
  W32 := isExternalRead
  W8 := Bits(4, width = 4)
  W9 := op === W8
  W10 := Bits(0, width = 32)
  operand1 := Mux(W9, W10, AM_mem_read_data_mux_regfile_0_read_port_num_0)
  W11 := Bits(1, width = 4)
  W12 := op > W11
  operand2 := Mux(W12, imm, AM_mem_read_data_mux_regfile_0_read_port_num_1)
  AM_memWData_regfile_0_thread_0_writeNum_0 := operand1 + operand2
  AM_memWData_regfile_0_thread_0_writeNum_1 := operand1 - operand2
  W13 := Bits(0, width = 4)
  W14 := op === W13
  W15 := Bits(2, width = 4)
  W16 := op === W15
  W17 := W14 | W16
  W18 := Bits(4, width = 4)
  W19 := op === W18
  adderSel := W17 | W19
  W20 := Bits(1, width = 4)
  W21 := op === W20
  W22 := Bits(3, width = 4)
  W23 := op === W22
  subtractSel := W21 | W23
  W24 := Bits(8, width = 4)
  isLoad := op === W24
  W25 := Bits(9, width = 4)
  isStore := op === W25
  W26 := Bits(1, width = 1)
  W27 := Bits(0, width = 1)
  memWrite := Mux(isStore, W26, W27)
  W28 := AM_mem_read_data_mux_regfile_0_read_port_num_0(9, 0)
  W29 := Cat(memWrite, W28, AM_mem_read_data_mux_regfile_0_read_port_num_1)
  io.dmemPort_0.reqBits := W29
  W30 := isLoad | isStore
  W91 := W30
  W31 := Bool(true)
  W35 := Bits(0, width = 1)
  W36 := AM_stage_thread_sel_id_0 === W35
  AM_inputIO_bits_mux_readAddr_0 := Mux(W36, io.readAddr_0.bits, io.readAddr_0.bits)
  W37 := Bits(0, width = 1)
  W38 := AM_stage_thread_sel_id_0 === W37
  AM_varLatIO_resp_bits_mux_imemPort_0 := Mux(W38, io.imemPort_0.respBits, io.imemPort_0.respBits)
  W39 := Bits(0, width = 1)
  W40 := AM_stage_thread_sel_id_0 === W39
  AM_memWData_regfile_0_thread_0_writeNum_2 := Mux(W40, io.dmemPort_0.respBits, io.dmemPort_0.respBits)
  W41 := Bits(0, width = 1)
  W42 := AM_stage_thread_sel_id_0 === W41
  AM_reg_mux_pcReg_0 := Mux(W42, pcReg_0, pcReg_0)
  W43 := Bits(0, width = 1)
  W44 := AM_stage_thread_sel_id_0 === W43
  AM_mem_read_data_mux_regfile_0_read_port_num_0 := Mux(W44, rs1Data, rs1Data)
  W45 := Bits(0, width = 1)
  W46 := AM_stage_thread_sel_id_0 === W45
  AM_mem_read_data_mux_regfile_0_read_port_num_1 := Mux(W46, rs2Data, rs2Data)
  W47 := Bits(0, width = 1)
  W48 := AM_stage_thread_sel_id_0 === W47
  AM_mem_read_data_mux_regfile_0_read_port_num_2 := Mux(W48, W33, W33)
  W49 := Bits(1, width = 1)
  W50 := AM_thread_sel_counter + W49
  W51 := Bool(true)
  W52 := Bits(1, width = 1)
  W53 := W50 === W52
  W54 := Bits(0, width = 1)
  AM_stage_thread_sel_id_0 := AM_thread_sel_counter
  W55 := Bits(0, width = 1)
  AM_perStageThreadSel_stage_0_thread_0 := AM_stage_thread_sel_id_0 === W55
  W57 := ~ io.readData_0.ready
  AM_iobusy_readData_0 := W56 & W57
  W58 := AM_perStageThreadSel_stage_0_thread_0 & AM_iobusy_readData_0
  W60 := ~ io.readAddr_0.valid
  AM_iobusy_readAddr_0 := W60 & W59
  W61 := AM_perStageThreadSel_stage_0_thread_0 & AM_iobusy_readAddr_0
  AM_varLatIO_busy_imemPort_0 := io.imemPort_0.respPending & AM_perStageThreadSel_stage_0_thread_0
  AM_varLatIO_busy_dmemPort_0 := io.dmemPort_0.respPending & AM_perStageThreadSel_stage_0_thread_0
  W62 := Bool(true)
  AM_stage_NoRAW_0 := W62
  W63 := Bool(true)
  W64 := ~ W61
  W65 := W63 & W64
  W66 := ~ W58
  W67 := W65 & W66
  AM_stage_NoIOBusy_0 := W67
  W68 := Bool(false)
  W69 := W68 | AM_varLatIO_busy_imemPort_0
  W70 := W69 | AM_varLatIO_busy_dmemPort_0
  AM_stage_kill_0 := W70
  W71 := ~ AM_stage_kill_0
  W72 := W31 & W71
  W73 := AM_stage_NoRAW_0 & AM_stage_NoIOBusy_0
  W74 := W72 & W73
  AM_stage_valid_0 := W74
  W75 := Bool(false)
  AM_stage_stall_0 := W75
  W76 := isNotJmp & AM_stage_valid_0
  W77 := isJmp & AM_stage_valid_0
  W78 := adderSel & AM_stage_valid_0
  W79 := subtractSel & AM_stage_valid_0
  W80 := isLoad & AM_stage_valid_0
  W81 := AM_stage_NoRAW_0 & W31
  W83 := W82 & W81
  W84 := AM_stage_NoRAW_0 & W31
  W86 := W85 & W84
  W87 := AM_stage_NoRAW_0 & W31
  W89 := W88 & W87
  W90 := AM_stage_NoRAW_0 & W31
  W92 := W91 & W90
  W93 := ~ W58
  W95 := W94 & W93
  W96 := ~ AM_varLatIO_busy_imemPort_0
  W98 := W97 & W96
  W99 := ~ AM_varLatIO_busy_dmemPort_0
  W101 := W100 & W99
  W102 := ~ W61
  W104 := W103 & W102
  W105 := ~ AM_varLatIO_busy_imemPort_0
  W107 := W106 & W105
  W108 := ~ AM_varLatIO_busy_dmemPort_0
  W110 := W109 & W108
  W111 := ~ W61
  W113 := W112 & W111
  W114 := ~ W58
  W116 := W115 & W114
  W117 := ~ AM_varLatIO_busy_dmemPort_0
  W119 := W118 & W117
  W120 := ~ W61
  W122 := W121 & W120
  W123 := ~ W58
  W125 := W124 & W123
  W126 := ~ AM_varLatIO_busy_imemPort_0
  W128 := W127 & W126
  W129 := W51 & AM_stage_valid_0
  W130 := W53 & AM_stage_valid_0
  W131 := ~ AM_stage_stall_0
  W132 := W76 & W131
  W133 := ~ AM_stage_stall_0
  W134 := W77 & W133
  W135 := ~ AM_stage_stall_0
  W136 := W78 & W135
  W137 := ~ AM_stage_stall_0
  W138 := W79 & W137
  W139 := ~ AM_stage_stall_0
  W140 := W80 & W139
  W141 := ~ AM_stage_stall_0
  W143 := W142 & W141
  W144 := ~ AM_stage_stall_0
  W146 := W145 & W144
  W147 := ~ AM_stage_stall_0
  W149 := W148 & W147
  W150 := ~ AM_stage_stall_0
  W152 := W151 & W150
  W153 := ~ AM_stage_stall_0
  W154 := W129 & W153
  W155 := ~ AM_stage_stall_0
  W156 := W130 & W155
  AM_regWEn_pcReg_0_thread_0_writeNum_0 := W132 & AM_perStageThreadSel_stage_0_thread_0
  AM_regWEn_pcReg_0_thread_0_writeNum_1 := W134 & AM_perStageThreadSel_stage_0_thread_0
  AM_memWEn_regfile_0_thread_0_writeNum_0 := W136 & AM_perStageThreadSel_stage_0_thread_0
  AM_memWEn_regfile_0_thread_0_writeNum_1 := W138 & AM_perStageThreadSel_stage_0_thread_0
  AM_memWEn_regfile_0_thread_0_writeNum_2 := W140 & AM_perStageThreadSel_stage_0_thread_0
  W158 := W157 & AM_perStageThreadSel_stage_0_thread_0
  W160 := W159 & AM_perStageThreadSel_stage_0_thread_0
  W162 := W161 & AM_perStageThreadSel_stage_0_thread_0
  W164 := W163 & AM_perStageThreadSel_stage_0_thread_0
  io.readData_0.valid := W158
  io.readData_0.bits := AM_mem_read_data_mux_regfile_0_read_port_num_2
  io.readAddr_0.ready := W160
  io.imemPort_0.reqValid := W162
  io.dmemPort_0.reqValid := W164
  W56 := W32
  W59 := W34
  W82 := W56
  W85 := W59
  W94 := W86
  W97 := W95
  W100 := W98
  W103 := W83
  W106 := W104
  W109 := W107
  W112 := W89
  W115 := W113
  W118 := W116
  W121 := W92
  W124 := W122
  W127 := W125
  W142 := W110
  W145 := W101
  W148 := W119
  W151 := W128
  W157 := W143
  W159 := W146
  W161 := W149
  W163 := W152
  val pcReg_0_reg = Reg(init = Bits(0, width = 4))
  pcReg_0 := pcReg_0_reg
  when(AM_regWEn_pcReg_0_thread_0_writeNum_0){
    pcReg_0_reg := AM_regWData_pcReg_0_thread_0_writeNum_0
  }
  when(AM_regWEn_pcReg_0_thread_0_writeNum_1){
    pcReg_0_reg := AM_regWData_pcReg_0_thread_0_writeNum_1
  }
  val regfile_0 = Mem(Bits(width = 32), 16)
  rs1Data := regfile_0.read(rs1)
  rs2Data := regfile_0.read(rs2)
  W33 := regfile_0.read(AM_inputIO_bits_mux_readAddr_0)
  when(AM_memWEn_regfile_0_thread_0_writeNum_0){
    regfile_0.write(AM_memWAddr_regfile_0_thread_0_writeNum_2, AM_memWData_regfile_0_thread_0_writeNum_0)
  }
  when(AM_memWEn_regfile_0_thread_0_writeNum_1){
    regfile_0.write(AM_memWAddr_regfile_0_thread_0_writeNum_2, AM_memWData_regfile_0_thread_0_writeNum_1)
  }
  when(AM_memWEn_regfile_0_thread_0_writeNum_2){
    regfile_0.write(AM_memWAddr_regfile_0_thread_0_writeNum_2, AM_memWData_regfile_0_thread_0_writeNum_2)
  }
  val AM_thread_sel_counter_reg = Reg(init = Bits(0, width = 1))
  AM_thread_sel_counter := AM_thread_sel_counter_reg
  when(W154){
    AM_thread_sel_counter_reg := W50
  }
  when(W156){
    AM_thread_sel_counter_reg := W54
  }
}
