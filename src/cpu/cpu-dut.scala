package Cpu
import Chisel._
import Common._
 
class Cpu extends Module {
  val io = new Bundle {
    var readData_0 = new DecoupledIO(Bits(width = 32))
    var readAddr_0 = new DecoupledIO(Bits(width = 4)).flip
    var readData_1 = new DecoupledIO(Bits(width = 32))
    var readData_2 = new DecoupledIO(Bits(width = 32))
    var readData_3 = new DecoupledIO(Bits(width = 32))
    var readAddr_1 = new DecoupledIO(Bits(width = 4)).flip
    var readAddr_2 = new DecoupledIO(Bits(width = 4)).flip
    var readAddr_3 = new DecoupledIO(Bits(width = 4)).flip
    var imemPort_0 = new VarLatIO(4, 32)
    var dmemPort_0 = new VarLatIO(43, 32)
    var imemPort_1 = new VarLatIO(4, 32)
    var imemPort_2 = new VarLatIO(4, 32)
    var imemPort_3 = new VarLatIO(4, 32)
    var dmemPort_1 = new VarLatIO(43, 32)
    var dmemPort_2 = new VarLatIO(43, 32)
    var dmemPort_3 = new VarLatIO(43, 32)
  }
  val pcReg_0 = Bits()
  val W0 = Bits()
  val pcSpec = Bits()
  val W1 = Bits()
  val AM_regWData_pcReg_0_thread_3_writeNum_0 = Bits()
  val W2 = Bool()
  val inst = Bits()
  val rs1 = Bits()
  val rs2 = Bits()
  val AM_memWAddr_regfile_0_thread_3_writeNum_2 = Bits()
  val op = Bits()
  val imm = Bits()
  val AM_regWData_pcReg_0_thread_3_writeNum_1 = Bits()
  val W3 = Bits()
  val isJmp = Bool()
  val W4 = Bits()
  val isNotJmp = Bool()
  val rs1Data = Bits()
  val rs2Data = Bits()
  val W5 = Bool()
  val W6 = Bool()
  val W7 = Bool()
  val W8 = Bits()
  val isExternalRead = Bool()
  val W9 = Bits()
  val W10 = Bool()
  val W11 = Bits()
  val operand1 = Bits()
  val W12 = Bits()
  val W13 = Bool()
  val operand2 = Bits()
  val AM_memWData_regfile_0_thread_3_writeNum_0 = Bits()
  val AM_memWData_regfile_0_thread_3_writeNum_1 = Bits()
  val W14 = Bits()
  val W15 = Bool()
  val W16 = Bits()
  val W17 = Bool()
  val W18 = Bool()
  val W19 = Bits()
  val W20 = Bool()
  val adderSel = Bool()
  val W21 = Bits()
  val W22 = Bool()
  val W23 = Bits()
  val W24 = Bool()
  val subtractSel = Bool()
  val W25 = Bits()
  val isLoad = Bool()
  val W26 = Bits()
  val isStore = Bool()
  val W27 = Bits()
  val W28 = Bits()
  val memWrite = Bits()
  val W29 = Bits()
  val W30 = Bits()
  val W31 = Bool()
  val AM_stage_valid_0 = Bool()
  val AM_stage_valid_1 = Bool()
  val AM_stage_valid_2 = Bool()
  val AM_stage_valid_3 = Bool()
  val W32 = Bool()
  val AM_prev_stage_valid_reg1 = Bool()
  val W33 = Bool()
  val AM_prev_stage_valid_reg2 = Bool()
  val W34 = Bool()
  val AM_prev_stage_valid_reg3 = Bool()
  val W35 = Bool()
  val AM_stage_stall_0 = Bool()
  val AM_stage_stall_1 = Bool()
  val AM_stage_stall_2 = Bool()
  val AM_stage_stall_3 = Bool()
  val AM_stage_kill_0 = Bool()
  val AM_stage_kill_1 = Bool()
  val AM_stage_kill_2 = Bool()
  val AM_stage_kill_3 = Bool()
  val AM_stage_NoRAW_0 = Bool()
  val AM_stage_NoRAW_1 = Bool()
  val AM_stage_NoRAW_2 = Bool()
  val AM_stage_NoRAW_3 = Bool()
  val AM_stage_NoIOBusy_0 = Bool()
  val AM_stage_NoIOBusy_1 = Bool()
  val AM_stage_NoIOBusy_2 = Bool()
  val AM_stage_NoIOBusy_3 = Bool()
  val AM_stage_thread_sel_id_0 = Bits(width=3)
  val AM_stage_thread_sel_id_1 = Bits()
  val W36 = Bool()
  val AM_stage_thread_sel_id_2 = Bits()
  val W37 = Bool()
  val AM_stage_thread_sel_id_3 = Bits()
  val W38 = Bool()
  val AM_global_stall = Bool()
  val W39 = Bool()
  val AM_pipe_reg_0_stage_2 = Bool()
  val W40 = Bool()
  val W41 = Bool()
  val AM_pipe_reg_1_stage_2valid = Bool()
  val W42 = Bool()
  val W43 = Bits()
  val AM_pipe_reg_2_stage_2bits = Bits()
  val W44 = Bool()
  val W45 = Bool()
  val AM_pipe_reg_3_stage_2isNotJmp = Bool()
  val W46 = Bool()
  val W47 = Bits()
  val AM_pipe_reg_4_stage_0pcPlus4 = Bits()
  val W48 = Bool()
  val AM_pipe_reg_5_stage_1pcPlus4 = Bits()
  val W49 = Bool()
  val AM_pipe_reg_6_stage_2pcPlus4 = Bits()
  val W50 = Bool()
  val W51 = Bool()
  val AM_pipe_reg_7_stage_2isJmp = Bool()
  val W52 = Bool()
  val W53 = Bits()
  val AM_pipe_reg_8_stage_2jmpTarget = Bits()
  val W54 = Bool()
  val W55 = Bits()
  val AM_pipe_reg_9_stage_2rd = Bits()
  val W56 = Bool()
  val W57 = Bool()
  val AM_pipe_reg_10_stage_2adderSel = Bool()
  val W58 = Bool()
  val W59 = Bool()
  val AM_pipe_reg_11_stage_2subtractSel = Bool()
  val W60 = Bool()
  val W61 = Bool()
  val AM_pipe_reg_12_stage_1isLoad = Bool()
  val W62 = Bool()
  val W63 = Bool()
  val AM_pipe_reg_13_stage_2reqValid = Bool()
  val W64 = Bool()
  val W65 = Bits()
  val AM_pipe_reg_14_stage_1 = Bits()
  val W66 = Bool()
  val W67 = Bits()
  val AM_pipe_reg_15_stage_1 = Bits()
  val W68 = Bool()
  val W69 = Bits()
  val AM_pipe_reg_16_stage_1 = Bits()
  val W70 = Bool()
  val W71 = Bits()
  val AM_pipe_reg_17_stage_1 = Bits()
  val W72 = Bool()
  val W73 = Bits()
  val AM_pipe_reg_18_stage_1 = Bits()
  val W74 = Bool()
  val W75 = Bits()
  val AM_pipe_reg_19_stage_1imm = Bits()
  val W76 = Bool()
  val W77 = Bits()
  val AM_pipe_reg_20_stage_0inst = Bits()
  val W78 = Bool()
  val W79 = Bits()
  val AM_pipe_reg_21_stage_2W30 = Bits()
  val W80 = Bool()
  val W81 = Bits()
  val AM_pipe_reg_22_stage_1 = Bits()
  val W82 = Bool()
  val W83 = Bits()
  val AM_pipe_reg_23_stage_1 = Bits()
  val W84 = Bool()
  val W85 = Bool()
  val AM_pipe_reg_24_stage_2 = Bool()
  val W86 = Bool()
  val W87 = Bits()
  val AM_pipe_reg_25_stage_2 = Bits()
  val W88 = Bool()
  val W89 = Bool()
  val AM_pipe_reg_26_stage_2 = Bool()
  val W90 = Bool()
  val W91 = Bits()
  val AM_pipe_reg_27_stage_2 = Bits()
  val W92 = Bool()
  val W93 = Bits()
  val AM_pipe_reg_28_stage_2 = Bits()
  val W94 = Bool()
  val W95 = Bits()
  val AM_pipe_reg_29_stage_1 = Bits()
  val W96 = Bool()
  val W97 = Bits()
  val AM_pipe_reg_30_stage_1 = Bits()
  val W98 = Bool()
  val W99 = Bool()
  val AM_pipe_reg_31_stage_1W15 = Bool()
  val W100 = Bool()
  val W101 = Bool()
  val AM_pipe_reg_32_stage_1W17 = Bool()
  val W102 = Bool()
  val W103 = Bool()
  val AM_pipe_reg_33_stage_1isStore = Bool()
  val W104 = Bool()
  val W105 = Bits()
  val AM_pipe_reg_34_stage_1 = Bits()
  val W106 = Bool()
  val W107 = Bits()
  val AM_pipe_reg_35_stage_1 = Bits()
  val W108 = Bool()
  val W109 = Bool()
  val W110 = Bits()
  val W111 = Bool()
  val W112 = Bool()
  val W113 = Bits()
  val W114 = Bool()
  val W115 = Bits()
  val W116 = Bool()
  val W117 = Bits()
  val W118 = Bool()
  val W119 = Bits()
  val W120 = Bool()
  val W121 = Bits()
  val W122 = Bool()
  val W123 = Bits()
  val W124 = Bits()
  val W125 = Bool()
  val W126 = Bits()
  val W127 = Bool()
  val W128 = Bits()
  val W129 = Bool()
  val W130 = Bits()
  val W131 = Bool()
  val W132 = Bits()
  val W133 = Bits()
  val W134 = Bits()
  val AM_inputIO_bits_mux_readAddr_0 = Bits()
  val W135 = Bits()
  val W136 = Bool()
  val W137 = Bits()
  val W138 = Bool()
  val W139 = Bits()
  val W140 = Bool()
  val W141 = Bits()
  val W142 = Bool()
  val W143 = Bits()
  val W144 = Bits()
  val W145 = Bits()
  val AM_varLatIO_resp_bits_mux_imemPort_0 = Bits()
  val W146 = Bits()
  val W147 = Bool()
  val W148 = Bits()
  val W149 = Bool()
  val W150 = Bits()
  val W151 = Bool()
  val W152 = Bits()
  val W153 = Bool()
  val W154 = Bits()
  val W155 = Bits()
  val W156 = Bits()
  val AM_memWData_regfile_0_thread_3_writeNum_2 = Bits()
  val pcReg_1 = Bits()
  val pcReg_2 = Bits()
  val pcReg_3 = Bits()
  val W157 = Bits()
  val W158 = Bits()
  val W159 = Bits()
  val W160 = Bits()
  val W161 = Bits()
  val W162 = Bits()
  val W163 = Bits()
  val W164 = Bits()
  val W165 = Bits()
  val W166 = Bits()
  val W167 = Bool()
  val W168 = Bits()
  val W169 = Bool()
  val W170 = Bits()
  val W171 = Bool()
  val W172 = Bits()
  val W173 = Bool()
  val W174 = Bits()
  val W175 = Bits()
  val W176 = Bits()
  val AM_reg_mux_pcReg_0 = Bits()
  val W177 = Bits()
  val W178 = Bool()
  val W179 = Bits()
  val W180 = Bool()
  val W181 = Bits()
  val W182 = Bool()
  val W183 = Bits()
  val W184 = Bool()
  val W185 = Bits()
  val W186 = Bits()
  val W187 = Bits()
  val AM_mem_read_data_mux_regfile_0_read_port_num_0 = Bits()
  val W188 = Bits()
  val W189 = Bool()
  val W190 = Bits()
  val W191 = Bool()
  val W192 = Bits()
  val W193 = Bool()
  val W194 = Bits()
  val W195 = Bool()
  val W196 = Bits()
  val W197 = Bits()
  val W198 = Bits()
  val AM_mem_read_data_mux_regfile_0_read_port_num_1 = Bits()
  val W199 = Bits()
  val W200 = Bool()
  val W201 = Bits()
  val W202 = Bool()
  val W203 = Bits()
  val W204 = Bool()
  val W205 = Bits()
  val W206 = Bool()
  val W207 = Bits()
  val W208 = Bits()
  val W209 = Bits()
  val AM_mem_read_data_mux_regfile_0_read_port_num_2 = Bits()
  val AM_thread_sel_counter = Bits()
  val W210 = Bits()
  val W211 = Bits()
  val W212 = Bool()
  val W213 = Bits()
  val W214 = Bool()
  val W215 = Bits()
  val W216 = Bits()
  val AM_perStageThreadSel_stage_0_thread_0 = Bool()
  val W217 = Bits()
  val AM_perStageThreadSel_stage_0_thread_1 = Bool()
  val W218 = Bits()
  val AM_perStageThreadSel_stage_0_thread_2 = Bool()
  val W219 = Bits()
  val AM_perStageThreadSel_stage_0_thread_3 = Bool()
  val W220 = Bits()
  val AM_perStageThreadSel_stage_1_thread_0 = Bool()
  val W221 = Bits()
  val AM_perStageThreadSel_stage_1_thread_1 = Bool()
  val W222 = Bits()
  val AM_perStageThreadSel_stage_1_thread_2 = Bool()
  val W223 = Bits()
  val AM_perStageThreadSel_stage_1_thread_3 = Bool()
  val W224 = Bits()
  val AM_perStageThreadSel_stage_2_thread_0 = Bool()
  val W225 = Bits()
  val AM_perStageThreadSel_stage_2_thread_1 = Bool()
  val W226 = Bits()
  val AM_perStageThreadSel_stage_2_thread_2 = Bool()
  val W227 = Bits()
  val AM_perStageThreadSel_stage_2_thread_3 = Bool()
  val W228 = Bits()
  val AM_perStageThreadSel_stage_3_thread_0 = Bool()
  val W229 = Bits()
  val AM_perStageThreadSel_stage_3_thread_1 = Bool()
  val W230 = Bits()
  val AM_perStageThreadSel_stage_3_thread_2 = Bool()
  val W231 = Bits()
  val AM_perStageThreadSel_stage_3_thread_3 = Bool()
  val W232 = Bool()
  val W233 = Bool()
  val W234 = Bool()
  val AM_RAW_hazard_0_pcReg_0_stage_1_writeNum_0 = Bool()
  val W235 = Bool()
  val W236 = Bool()
  val W237 = Bool()
  val AM_RAW_hazard_1_pcReg_0_stage_2_writeNum_0 = Bool()
  val W238 = Bool()
  val W239 = Bool()
  val W240 = Bool()
  val AM_RAW_hazard_2_pcReg_0_stage_3_writeNum_0 = Bool()
  val W241 = Bool()
  val W242 = Bool()
  val W243 = Bool()
  val AM_RAW_hazard_3_pcReg_0_stage_1_writeNum_1 = Bool()
  val W244 = Bool()
  val W245 = Bool()
  val W246 = Bool()
  val AM_RAW_hazard_4_pcReg_0_stage_2_writeNum_1 = Bool()
  val W247 = Bool()
  val W248 = Bool()
  val W249 = Bool()
  val AM_RAW_hazard_5_pcReg_0_stage_3_writeNum_1 = Bool()
  val W250 = Bool()
  val W251 = Bits()
  val W252 = Bool()
  val W253 = Bool()
  val W254 = Bool()
  val W255 = Bool()
  val W256 = Bool()
  val AM_RAW_hazard_6_regfile_0_readNum_0_stage_3_writeNum_0 = Bool()
  val W257 = Bool()
  val W258 = Bits()
  val W259 = Bool()
  val W260 = Bool()
  val W261 = Bool()
  val W262 = Bool()
  val W263 = Bool()
  val AM_RAW_hazard_7_regfile_0_readNum_1_stage_3_writeNum_0 = Bool()
  val W264 = Bool()
  val W265 = Bits()
  val W266 = Bool()
  val W267 = Bool()
  val W268 = Bool()
  val W269 = Bool()
  val W270 = Bool()
  val AM_RAW_hazard_8_regfile_0_readNum_2_stage_3_writeNum_0 = Bool()
  val W271 = Bool()
  val W272 = Bits()
  val W273 = Bool()
  val W274 = Bool()
  val W275 = Bool()
  val W276 = Bool()
  val W277 = Bool()
  val AM_RAW_hazard_9_regfile_0_readNum_0_stage_3_writeNum_1 = Bool()
  val W278 = Bool()
  val W279 = Bits()
  val W280 = Bool()
  val W281 = Bool()
  val W282 = Bool()
  val W283 = Bool()
  val W284 = Bool()
  val AM_RAW_hazard_10_regfile_0_readNum_1_stage_3_writeNum_1 = Bool()
  val W285 = Bool()
  val W286 = Bits()
  val W287 = Bool()
  val W288 = Bool()
  val W289 = Bool()
  val W290 = Bool()
  val W291 = Bool()
  val AM_RAW_hazard_11_regfile_0_readNum_2_stage_3_writeNum_1 = Bool()
  val W292 = Bool()
  val W293 = Bits()
  val W294 = Bool()
  val W295 = Bool()
  val W296 = Bool()
  val W297 = Bool()
  val W298 = Bool()
  val AM_RAW_hazard_12_regfile_0_readNum_0_stage_3_writeNum_2 = Bool()
  val W299 = Bool()
  val W300 = Bits()
  val W301 = Bool()
  val W302 = Bool()
  val W303 = Bool()
  val W304 = Bool()
  val W305 = Bool()
  val AM_RAW_hazard_13_regfile_0_readNum_1_stage_3_writeNum_2 = Bool()
  val W306 = Bool()
  val W307 = Bits()
  val W308 = Bool()
  val W309 = Bool()
  val W310 = Bool()
  val W311 = Bool()
  val W312 = Bool()
  val AM_RAW_hazard_14_regfile_0_readNum_2_stage_3_writeNum_2 = Bool()
  val W313 = Bool()
  val W314 = Bool()
  val AM_iobusy_readData_0 = Bool()
  val W315 = Bool()
  val W316 = Bool()
  val W317 = Bool()
  val AM_iobusy_readData_1 = Bool()
  val W318 = Bool()
  val W319 = Bool()
  val W320 = Bool()
  val AM_iobusy_readData_2 = Bool()
  val W321 = Bool()
  val W322 = Bool()
  val W323 = Bool()
  val AM_iobusy_readData_3 = Bool()
  val W324 = Bool()
  val W325 = Bool()
  val W326 = Bool()
  val AM_iobusy_readAddr_0 = Bool()
  val W327 = Bool()
  val W328 = Bool()
  val W329 = Bool()
  val AM_iobusy_readAddr_1 = Bool()
  val W330 = Bool()
  val W331 = Bool()
  val W332 = Bool()
  val AM_iobusy_readAddr_2 = Bool()
  val W333 = Bool()
  val W334 = Bool()
  val W335 = Bool()
  val AM_iobusy_readAddr_3 = Bool()
  val W336 = Bool()
  val AM_varLatIO_busy_imemPort_0 = Bool()
  val AM_varLatIO_busy_imemPort_1 = Bool()
  val AM_varLatIO_busy_imemPort_2 = Bool()
  val AM_varLatIO_busy_imemPort_3 = Bool()
  val AM_varLatIO_busy_dmemPort_0 = Bool()
  val AM_varLatIO_busy_dmemPort_1 = Bool()
  val AM_varLatIO_busy_dmemPort_2 = Bool()
  val AM_varLatIO_busy_dmemPort_3 = Bool()
  val W337 = Bool()
  val W338 = Bool()
  val W339 = Bool()
  val W340 = Bool()
  val W341 = Bool()
  val W342 = Bool()
  val W343 = Bool()
  val W344 = Bool()
  val W345 = Bool()
  val W346 = Bool()
  val W347 = Bool()
  val W348 = Bool()
  val W349 = Bool()
  val W350 = Bool()
  val W351 = Bool()
  val W352 = Bool()
  val W353 = Bool()
  val W354 = Bool()
  val W355 = Bool()
  val W356 = Bool()
  val W357 = Bool()
  val W358 = Bool()
  val W359 = Bool()
  val W360 = Bool()
  val W361 = Bool()
  val W362 = Bool()
  val W363 = Bool()
  val W364 = Bool()
  val W365 = Bool()
  val W366 = Bool()
  val W367 = Bool()
  val W368 = Bool()
  val W369 = Bool()
  val W370 = Bool()
  val W371 = Bool()
  val W372 = Bool()
  val W373 = Bool()
  val W374 = Bool()
  val W375 = Bool()
  val W376 = Bool()
  val W377 = Bool()
  val W378 = Bool()
  val W379 = Bool()
  val W380 = Bool()
  val W381 = Bool()
  val W382 = Bool()
  val W383 = Bool()
  val W384 = Bool()
  val W385 = Bool()
  val W386 = Bool()
  val W387 = Bool()
  val W388 = Bool()
  val W389 = Bool()
  val W390 = Bool()
  val W391 = Bool()
  val W392 = Bool()
  val W393 = Bool()
  val W394 = Bool()
  val W395 = Bool()
  val W396 = Bool()
  val W397 = Bool()
  val W398 = Bool()
  val W399 = Bool()
  val W400 = Bool()
  val W401 = Bool()
  val W402 = Bool()
  val W403 = Bool()
  val W404 = Bool()
  val W405 = Bool()
  val W406 = Bool()
  val W407 = Bool()
  val W408 = Bool()
  val W409 = Bool()
  val W410 = Bool()
  val W411 = Bool()
  val W412 = Bool()
  val W413 = Bool()
  val W414 = Bool()
  val W415 = Bool()
  val W416 = Bool()
  val W417 = Bool()
  val W418 = Bool()
  val W419 = Bool()
  val W420 = Bool()
  val W421 = Bool()
  val W422 = Bool()
  val W423 = Bool()
  val W424 = Bool()
  val W425 = Bool()
  val W426 = Bool()
  val W427 = Bool()
  val W428 = Bool()
  val W429 = Bool()
  val W430 = Bool()
  val W431 = Bool()
  val W432 = Bool()
  val W433 = Bool()
  val W434 = Bool()
  val W435 = Bool()
  val W436 = Bool()
  val W437 = Bool()
  val W438 = Bool()
  val W439 = Bool()
  val W440 = Bool()
  val W441 = Bool()
  val W442 = Bool()
  val W443 = Bool()
  val W444 = Bool()
  val W445 = Bool()
  val W446 = Bool()
  val W447 = Bool()
  val W448 = Bool()
  val W449 = Bool()
  val W450 = Bool()
  val W451 = Bool()
  val W452 = Bool()
  val W453 = Bool()
  val W454 = Bool()
  val W455 = Bool()
  val W456 = Bool()
  val W457 = Bool()
  val W458 = Bool()
  val W459 = Bool()
  val W460 = Bool()
  val W461 = Bool()
  val W462 = Bool()
  val W463 = Bool()
  val W464 = Bool()
  val W465 = Bool()
  val W466 = Bool()
  val W467 = Bool()
  val W468 = Bool()
  val W469 = Bool()
  val W470 = Bool()
  val W471 = Bool()
  val W472 = Bool()
  val W473 = Bool()
  val W474 = Bool()
  val W475 = Bool()
  val W476 = Bool()
  val W477 = Bool()
  val W478 = Bool()
  val W479 = Bool()
  val W480 = Bool()
  val W481 = Bool()
  val W482 = Bool()
  val W483 = Bool()
  val W484 = Bool()
  val W485 = Bool()
  val W486 = Bool()
  val W487 = Bool()
  val W488 = Bool()
  val W489 = Bool()
  val W490 = Bool()
  val W491 = Bool()
  val W492 = Bool()
  val W493 = Bool()
  val W494 = Bool()
  val W495 = Bool()
  val W496 = Bool()
  val W497 = Bool()
  val W498 = Bool()
  val W499 = Bool()
  val W500 = Bool()
  val W501 = Bool()
  val W502 = Bool()
  val W503 = Bool()
  val W504 = Bool()
  val W505 = Bool()
  val W506 = Bool()
  val W507 = Bool()
  val W508 = Bool()
  val W509 = Bool()
  val W510 = Bool()
  val W511 = Bool()
  val W512 = Bool()
  val W513 = Bool()
  val W514 = Bool()
  val W515 = Bool()
  val W516 = Bool()
  val W517 = Bool()
  val W518 = Bool()
  val W519 = Bool()
  val W520 = Bool()
  val W521 = Bool()
  val W522 = Bool()
  val W523 = Bool()
  val W524 = Bool()
  val W525 = Bool()
  val W526 = Bool()
  val W527 = Bool()
  val W528 = Bool()
  val W529 = Bool()
  val W530 = Bool()
  val W531 = Bool()
  val W532 = Bool()
  val W533 = Bool()
  val W534 = Bool()
  val W535 = Bool()
  val W536 = Bool()
  val W537 = Bool()
  val W538 = Bool()
  val W539 = Bool()
  val W540 = Bool()
  val W541 = Bool()
  val W542 = Bool()
  val W543 = Bool()
  val W544 = Bool()
  val W545 = Bool()
  val W546 = Bool()
  val W547 = Bool()
  val W548 = Bool()
  val W549 = Bool()
  val W550 = Bool()
  val W551 = Bool()
  val W552 = Bool()
  val W553 = Bool()
  val W554 = Bool()
  val W555 = Bool()
  val W556 = Bool()
  val W557 = Bool()
  val W558 = Bool()
  val W559 = Bool()
  val W560 = Bool()
  val W561 = Bool()
  val W562 = Bool()
  val W563 = Bool()
  val W564 = Bool()
  val W565 = Bool()
  val W566 = Bool()
  val W567 = Bool()
  val W568 = Bool()
  val W569 = Bool()
  val W570 = Bool()
  val W571 = Bool()
  val W572 = Bool()
  val W573 = Bool()
  val W574 = Bool()
  val W575 = Bool()
  val W576 = Bool()
  val W577 = Bool()
  val W578 = Bool()
  val W579 = Bool()
  val W580 = Bool()
  val W581 = Bool()
  val W582 = Bool()
  val W583 = Bool()
  val W584 = Bool()
  val W585 = Bool()
  val W586 = Bool()
  val W587 = Bool()
  val W588 = Bool()
  val W589 = Bool()
  val W590 = Bool()
  val W591 = Bool()
  val W592 = Bool()
  val W593 = Bool()
  val W594 = Bool()
  val W595 = Bool()
  val W596 = Bool()
  val W597 = Bool()
  val W598 = Bool()
  val W599 = Bool()
  val W600 = Bool()
  val W601 = Bool()
  val W602 = Bool()
  val W603 = Bool()
  val W604 = Bool()
  val W605 = Bool()
  val W606 = Bool()
  val W607 = Bool()
  val W608 = Bool()
  val W609 = Bool()
  val W610 = Bool()
  val W611 = Bool()
  val W612 = Bool()
  val W613 = Bool()
  val W614 = Bool()
  val W615 = Bool()
  val W616 = Bool()
  val W617 = Bool()
  val W618 = Bool()
  val W619 = Bool()
  val W620 = Bool()
  val W621 = Bool()
  val W622 = Bool()
  val W623 = Bool()
  val W624 = Bool()
  val W625 = Bool()
  val W626 = Bool()
  val W627 = Bool()
  val W628 = Bool()
  val W629 = Bool()
  val W630 = Bool()
  val W631 = Bool()
  val W632 = Bool()
  val W633 = Bool()
  val W634 = Bool()
  val W635 = Bool()
  val W636 = Bool()
  val W637 = Bool()
  val W638 = Bool()
  val W639 = Bool()
  val W640 = Bool()
  val W641 = Bool()
  val W642 = Bool()
  val W643 = Bool()
  val W644 = Bool()
  val W645 = Bool()
  val W646 = Bool()
  val W647 = Bool()
  val W648 = Bool()
  val W649 = Bool()
  val W650 = Bool()
  val W651 = Bool()
  val W652 = Bool()
  val W653 = Bool()
  val W654 = Bool()
  val W655 = Bool()
  val W656 = Bool()
  val W657 = Bool()
  val W658 = Bool()
  val W659 = Bool()
  val W660 = Bool()
  val W661 = Bool()
  val W662 = Bool()
  val W663 = Bool()
  val W664 = Bool()
  val W665 = Bool()
  val W666 = Bool()
  val W667 = Bool()
  val W668 = Bool()
  val W669 = Bool()
  val W670 = Bool()
  val W671 = Bool()
  val W672 = Bool()
  val W673 = Bool()
  val W674 = Bool()
  val AM_regWEn_pcReg_0_thread_0_writeNum_0 = Bool()
  val AM_regWEn_pcReg_0_thread_0_writeNum_1 = Bool()
  val AM_regWEn_pcReg_0_thread_1_writeNum_0 = Bool()
  val AM_regWEn_pcReg_0_thread_1_writeNum_1 = Bool()
  val AM_regWEn_pcReg_0_thread_2_writeNum_0 = Bool()
  val AM_regWEn_pcReg_0_thread_2_writeNum_1 = Bool()
  val AM_regWEn_pcReg_0_thread_3_writeNum_0 = Bool()
  val AM_regWEn_pcReg_0_thread_3_writeNum_1 = Bool()
  val AM_memWEn_regfile_0_thread_0_writeNum_0 = Bool()
  val AM_memWEn_regfile_0_thread_0_writeNum_1 = Bool()
  val AM_memWEn_regfile_0_thread_0_writeNum_2 = Bool()
  val AM_memWEn_regfile_0_thread_1_writeNum_0 = Bool()
  val AM_memWEn_regfile_0_thread_1_writeNum_1 = Bool()
  val AM_memWEn_regfile_0_thread_1_writeNum_2 = Bool()
  val AM_memWEn_regfile_0_thread_2_writeNum_0 = Bool()
  val AM_memWEn_regfile_0_thread_2_writeNum_1 = Bool()
  val AM_memWEn_regfile_0_thread_2_writeNum_2 = Bool()
  val AM_memWEn_regfile_0_thread_3_writeNum_0 = Bool()
  val AM_memWEn_regfile_0_thread_3_writeNum_1 = Bool()
  val AM_memWEn_regfile_0_thread_3_writeNum_2 = Bool()
  val W675 = Bool()
  val W676 = Bool()
  val W677 = Bool()
  val W678 = Bool()
  val W679 = Bool()
  val W680 = Bool()
  val W681 = Bool()
  val W682 = Bool()
  val W683 = Bool()
  val W684 = Bool()
  val W685 = Bool()
  val W686 = Bool()
  val W687 = Bool()
  val W688 = Bool()
  val W689 = Bool()
  val W690 = Bool()
  val W691 = Bool()
  val W692 = Bool()
  val W693 = Bool()
  val W694 = Bool()
  val W695 = Bool()
  val W696 = Bool()
  val W697 = Bool()
  val W698 = Bool()
  val W699 = Bool()
  val W700 = Bool()
  val W701 = Bool()
  val W702 = Bool()
  val W703 = Bool()
  val W704 = Bool()
  val W705 = Bool()
  val W706 = Bool()
  W0 := Bits(1, width = 4)
  pcSpec := AM_reg_mux_pcReg_0 + W0
  W1 := Bits(1, width = 4)
  W47 := AM_reg_mux_pcReg_0 + W1
  W2 := Bool(true)
  W112 := W2
  W113 := AM_reg_mux_pcReg_0
  W77 := AM_varLatIO_resp_bits_mux_imemPort_0
  rs1 := inst(11, 8)
  rs2 := inst(7, 4)
  W55 := AM_pipe_reg_18_stage_1(3, 0)
  op := inst(15, 12)
  W75 := inst(31, 16)
  W53 := imm(3, 0)
  W3 := Bits(6, width = 4)
  W51 := AM_pipe_reg_17_stage_1 === W3
  W4 := Bits(6, width = 4)
  W45 := AM_pipe_reg_16_stage_1 != W4
  W5 := Bool(true)
  W6 := Bool(true)
  W7 := Bool(true)
  W8 := Bits(7, width = 4)
  isExternalRead := AM_pipe_reg_22_stage_1 === W8
  W111 := isExternalRead
  W41 := isExternalRead
  W9 := Bits(4, width = 4)
  W10 := AM_pipe_reg_34_stage_1 === W9
  W11 := Bits(0, width = 32)
  operand1 := Mux(AM_pipe_reg_24_stage_2, W11, AM_pipe_reg_25_stage_2)
  W12 := Bits(1, width = 4)
  W13 := AM_pipe_reg_35_stage_1 > W12
  operand2 := Mux(AM_pipe_reg_26_stage_2, AM_pipe_reg_27_stage_2, AM_pipe_reg_28_stage_2)
  AM_memWData_regfile_0_thread_3_writeNum_0 := operand1 + operand2
  AM_memWData_regfile_0_thread_3_writeNum_1 := operand1 - operand2
  W14 := Bits(0, width = 4)
  W99 := op === W14
  W16 := Bits(2, width = 4)
  W101 := op === W16
  W18 := W15 | W17
  W19 := Bits(4, width = 4)
  W20 := AM_pipe_reg_23_stage_1 === W19
  W57 := W18 | W20
  W21 := Bits(1, width = 4)
  W22 := AM_pipe_reg_29_stage_1 === W21
  W23 := Bits(3, width = 4)
  W24 := AM_pipe_reg_30_stage_1 === W23
  W59 := W22 | W24
  W25 := Bits(8, width = 4)
  W61 := op === W25
  W26 := Bits(9, width = 4)
  W103 := op === W26
  W27 := Bits(1, width = 1)
  W28 := Bits(0, width = 1)
  memWrite := Mux(isStore, W27, W28)
  W29 := AM_mem_read_data_mux_regfile_0_read_port_num_0(9, 0)
  W79 := Cat(memWrite, W29, AM_mem_read_data_mux_regfile_0_read_port_num_1)
  W119 := W30
  W31 := isLoad | isStore
  W63 := W31
  W32 := Bool(true)
  W33 := Bool(true)
  W34 := Bool(true)
  W35 := Bool(true)
  W36 := Bool(true)
  W37 := Bool(true)
  W38 := Bool(true)
  W40 := Bool(true)
  W42 := Bool(true)
  W44 := Bool(true)
  W46 := Bool(true)
  W48 := Bool(true)
  W49 := Bool(true)
  W50 := Bool(true)
  W52 := Bool(true)
  W54 := Bool(true)
  W56 := Bool(true)
  W58 := Bool(true)
  W60 := Bool(true)
  W62 := Bool(true)
  W64 := Bool(true)
  W66 := Bool(true)
  W68 := Bool(true)
  W70 := Bool(true)
  W72 := Bool(true)
  W74 := Bool(true)
  W76 := Bool(true)
  W78 := Bool(true)
  W80 := Bool(true)
  W82 := Bool(true)
  W84 := Bool(true)
  W86 := Bool(true)
  W88 := Bool(true)
  W90 := Bool(true)
  W92 := Bool(true)
  W94 := Bool(true)
  W96 := Bool(true)
  W98 := Bool(true)
  W100 := Bool(true)
  W102 := Bool(true)
  W104 := Bool(true)
  W106 := Bool(true)
  W108 := Bool(true)
  W124 := Bits(0, width = 3)
  W125 := AM_stage_thread_sel_id_2 === W124
  W126 := Bits(1, width = 3)
  W127 := AM_stage_thread_sel_id_2 === W126
  W128 := Bits(2, width = 3)
  W129 := AM_stage_thread_sel_id_2 === W128
  W130 := Bits(3, width = 3)
  W131 := AM_stage_thread_sel_id_2 === W130
  W132 := Mux(W125, io.readAddr_0.bits, io.readAddr_0.bits)
  W133 := Mux(W127, io.readAddr_1.bits, W132)
  W134 := Mux(W129, io.readAddr_2.bits, W133)
  AM_inputIO_bits_mux_readAddr_0 := Mux(W131, io.readAddr_3.bits, W134)
  W135 := Bits(0, width = 3)
  W136 := AM_stage_thread_sel_id_0 === W135
  W137 := Bits(1, width = 3)
  W138 := AM_stage_thread_sel_id_0 === W137
  W139 := Bits(2, width = 3)
  W140 := AM_stage_thread_sel_id_0 === W139
  W141 := Bits(3, width = 3)
  W142 := AM_stage_thread_sel_id_0 === W141
  W143 := Mux(W136, io.imemPort_0.respBits, io.imemPort_0.respBits)
  W144 := Mux(W138, io.imemPort_1.respBits, W143)
  W145 := Mux(W140, io.imemPort_2.respBits, W144)
  AM_varLatIO_resp_bits_mux_imemPort_0 := Mux(W142, io.imemPort_3.respBits, W145)
  W146 := Bits(0, width = 3)
  W147 := AM_stage_thread_sel_id_3 === W146
  W148 := Bits(1, width = 3)
  W149 := AM_stage_thread_sel_id_3 === W148
  W150 := Bits(2, width = 3)
  W151 := AM_stage_thread_sel_id_3 === W150
  W152 := Bits(3, width = 3)
  W153 := AM_stage_thread_sel_id_3 === W152
  W154 := Mux(W147, io.dmemPort_0.respBits, io.dmemPort_0.respBits)
  W155 := Mux(W149, io.dmemPort_1.respBits, W154)
  W156 := Mux(W151, io.dmemPort_2.respBits, W155)
  AM_memWData_regfile_0_thread_3_writeNum_2 := Mux(W153, io.dmemPort_3.respBits, W156)
  W166 := Bits(0, width = 3)
  W167 := AM_stage_thread_sel_id_0 === W166
  W168 := Bits(1, width = 3)
  W169 := AM_stage_thread_sel_id_0 === W168
  W170 := Bits(2, width = 3)
  W171 := AM_stage_thread_sel_id_0 === W170
  W172 := Bits(3, width = 3)
  W173 := AM_stage_thread_sel_id_0 === W172
  W174 := Mux(W167, pcReg_0, pcReg_0)
  W175 := Mux(W169, pcReg_1, W174)
  W176 := Mux(W171, pcReg_2, W175)
  AM_reg_mux_pcReg_0 := Mux(W173, pcReg_3, W176)
  W177 := Bits(0, width = 3)
  W178 := AM_stage_thread_sel_id_2 === W177
  W179 := Bits(1, width = 3)
  W180 := AM_stage_thread_sel_id_2 === W179
  W181 := Bits(2, width = 3)
  W182 := AM_stage_thread_sel_id_2 === W181
  W183 := Bits(3, width = 3)
  W184 := AM_stage_thread_sel_id_2 === W183
  W185 := Mux(W178, rs1Data, rs1Data)
  W186 := Mux(W180, W157, W185)
  W187 := Mux(W182, W160, W186)
  AM_mem_read_data_mux_regfile_0_read_port_num_0 := Mux(W184, W163, W187)
  W188 := Bits(0, width = 3)
  W189 := AM_stage_thread_sel_id_2 === W188
  W190 := Bits(1, width = 3)
  W191 := AM_stage_thread_sel_id_2 === W190
  W192 := Bits(2, width = 3)
  W193 := AM_stage_thread_sel_id_2 === W192
  W194 := Bits(3, width = 3)
  W195 := AM_stage_thread_sel_id_2 === W194
  W196 := Mux(W189, rs2Data, rs2Data)
  W197 := Mux(W191, W158, W196)
  W198 := Mux(W193, W161, W197)
  AM_mem_read_data_mux_regfile_0_read_port_num_1 := Mux(W195, W164, W198)
  W199 := Bits(0, width = 3)
  W200 := AM_stage_thread_sel_id_2 === W199
  W201 := Bits(1, width = 3)
  W202 := AM_stage_thread_sel_id_2 === W201
  W203 := Bits(2, width = 3)
  W204 := AM_stage_thread_sel_id_2 === W203
  W205 := Bits(3, width = 3)
  W206 := AM_stage_thread_sel_id_2 === W205
  W207 := Mux(W200, W43, W43)
  W208 := Mux(W202, W159, W207)
  W209 := Mux(W204, W162, W208)
  AM_mem_read_data_mux_regfile_0_read_port_num_2 := Mux(W206, W165, W209)
  W210 := Bits(1, width = 3)
  W211 := AM_thread_sel_counter + W210
  W212 := Bool(true)
  W213 := Bits(4, width = 3)
  W214 := W211 === W213
  W215 := Bits(0, width = 3)
  AM_stage_thread_sel_id_0 := AM_thread_sel_counter
  W216 := Bits(0, width = 3)
  AM_perStageThreadSel_stage_0_thread_0 := AM_stage_thread_sel_id_0 === W216
  W217 := Bits(1, width = 3)
  AM_perStageThreadSel_stage_0_thread_1 := AM_stage_thread_sel_id_0 === W217
  W218 := Bits(2, width = 3)
  AM_perStageThreadSel_stage_0_thread_2 := AM_stage_thread_sel_id_0 === W218
  W219 := Bits(3, width = 3)
  AM_perStageThreadSel_stage_0_thread_3 := AM_stage_thread_sel_id_0 === W219
  W220 := Bits(0, width = 3)
  AM_perStageThreadSel_stage_1_thread_0 := AM_stage_thread_sel_id_1 === W220
  W221 := Bits(1, width = 3)
  AM_perStageThreadSel_stage_1_thread_1 := AM_stage_thread_sel_id_1 === W221
  W222 := Bits(2, width = 3)
  AM_perStageThreadSel_stage_1_thread_2 := AM_stage_thread_sel_id_1 === W222
  W223 := Bits(3, width = 3)
  AM_perStageThreadSel_stage_1_thread_3 := AM_stage_thread_sel_id_1 === W223
  W224 := Bits(0, width = 3)
  AM_perStageThreadSel_stage_2_thread_0 := AM_stage_thread_sel_id_2 === W224
  W225 := Bits(1, width = 3)
  AM_perStageThreadSel_stage_2_thread_1 := AM_stage_thread_sel_id_2 === W225
  W226 := Bits(2, width = 3)
  AM_perStageThreadSel_stage_2_thread_2 := AM_stage_thread_sel_id_2 === W226
  W227 := Bits(3, width = 3)
  AM_perStageThreadSel_stage_2_thread_3 := AM_stage_thread_sel_id_2 === W227
  W228 := Bits(0, width = 3)
  AM_perStageThreadSel_stage_3_thread_0 := AM_stage_thread_sel_id_3 === W228
  W229 := Bits(1, width = 3)
  AM_perStageThreadSel_stage_3_thread_1 := AM_stage_thread_sel_id_3 === W229
  W230 := Bits(2, width = 3)
  AM_perStageThreadSel_stage_3_thread_2 := AM_stage_thread_sel_id_3 === W230
  W231 := Bits(3, width = 3)
  AM_perStageThreadSel_stage_3_thread_3 := AM_stage_thread_sel_id_3 === W231
  W233 := AM_stage_thread_sel_id_0 === AM_stage_thread_sel_id_1
  W234 := W232 & W233
  AM_RAW_hazard_0_pcReg_0_stage_1_writeNum_0 := AM_stage_valid_1 & W234
  W236 := AM_stage_thread_sel_id_0 === AM_stage_thread_sel_id_2
  W237 := W235 & W236
  AM_RAW_hazard_1_pcReg_0_stage_2_writeNum_0 := AM_stage_valid_2 & W237
  W239 := AM_stage_thread_sel_id_0 === AM_stage_thread_sel_id_3
  W240 := W238 & W239
  AM_RAW_hazard_2_pcReg_0_stage_3_writeNum_0 := AM_stage_valid_3 & W240
  W242 := AM_stage_thread_sel_id_0 === AM_stage_thread_sel_id_1
  W243 := W241 & W242
  AM_RAW_hazard_3_pcReg_0_stage_1_writeNum_1 := AM_stage_valid_1 & W243
  W245 := AM_stage_thread_sel_id_0 === AM_stage_thread_sel_id_2
  W246 := W244 & W245
  AM_RAW_hazard_4_pcReg_0_stage_2_writeNum_1 := AM_stage_valid_2 & W246
  W248 := AM_stage_thread_sel_id_0 === AM_stage_thread_sel_id_3
  W249 := W247 & W248
  AM_RAW_hazard_5_pcReg_0_stage_3_writeNum_1 := AM_stage_valid_3 & W249
  W252 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W253 := AM_stage_valid_3 & W252
  W254 := AM_pipe_reg_14_stage_1 === W251
  W255 := W5 & W254
  W256 := W250 & W255
  AM_RAW_hazard_6_regfile_0_readNum_0_stage_3_writeNum_0 := W253 & W256
  W259 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W260 := AM_stage_valid_3 & W259
  W261 := AM_pipe_reg_15_stage_1 === W258
  W262 := W6 & W261
  W263 := W257 & W262
  AM_RAW_hazard_7_regfile_0_readNum_1_stage_3_writeNum_0 := W260 & W263
  W266 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W267 := AM_stage_valid_3 & W266
  W268 := AM_inputIO_bits_mux_readAddr_0 === W265
  W269 := W7 & W268
  W270 := W264 & W269
  AM_RAW_hazard_8_regfile_0_readNum_2_stage_3_writeNum_0 := W267 & W270
  W273 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W274 := AM_stage_valid_3 & W273
  W275 := AM_pipe_reg_14_stage_1 === W272
  W276 := W5 & W275
  W277 := W271 & W276
  AM_RAW_hazard_9_regfile_0_readNum_0_stage_3_writeNum_1 := W274 & W277
  W280 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W281 := AM_stage_valid_3 & W280
  W282 := AM_pipe_reg_15_stage_1 === W279
  W283 := W6 & W282
  W284 := W278 & W283
  AM_RAW_hazard_10_regfile_0_readNum_1_stage_3_writeNum_1 := W281 & W284
  W287 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W288 := AM_stage_valid_3 & W287
  W289 := AM_inputIO_bits_mux_readAddr_0 === W286
  W290 := W7 & W289
  W291 := W285 & W290
  AM_RAW_hazard_11_regfile_0_readNum_2_stage_3_writeNum_1 := W288 & W291
  W294 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W295 := AM_stage_valid_3 & W294
  W296 := AM_pipe_reg_14_stage_1 === W293
  W297 := W5 & W296
  W298 := W292 & W297
  AM_RAW_hazard_12_regfile_0_readNum_0_stage_3_writeNum_2 := W295 & W298
  W301 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W302 := AM_stage_valid_3 & W301
  W303 := AM_pipe_reg_15_stage_1 === W300
  W304 := W6 & W303
  W305 := W299 & W304
  AM_RAW_hazard_13_regfile_0_readNum_1_stage_3_writeNum_2 := W302 & W305
  W308 := AM_stage_thread_sel_id_2 === AM_stage_thread_sel_id_3
  W309 := AM_stage_valid_3 & W308
  W310 := AM_inputIO_bits_mux_readAddr_0 === W307
  W311 := W7 & W310
  W312 := W306 & W311
  AM_RAW_hazard_14_regfile_0_readNum_2_stage_3_writeNum_2 := W309 & W312
  W314 := ~ io.readData_0.ready
  AM_iobusy_readData_0 := W313 & W314
  W315 := AM_perStageThreadSel_stage_3_thread_0 & AM_iobusy_readData_0
  W317 := ~ io.readData_1.ready
  AM_iobusy_readData_1 := W316 & W317
  W318 := AM_perStageThreadSel_stage_3_thread_1 & AM_iobusy_readData_1
  W320 := ~ io.readData_2.ready
  AM_iobusy_readData_2 := W319 & W320
  W321 := AM_perStageThreadSel_stage_3_thread_2 & AM_iobusy_readData_2
  W323 := ~ io.readData_3.ready
  AM_iobusy_readData_3 := W322 & W323
  W324 := AM_perStageThreadSel_stage_3_thread_3 & AM_iobusy_readData_3
  W326 := ~ io.readAddr_0.valid
  AM_iobusy_readAddr_0 := W326 & W325
  W327 := AM_perStageThreadSel_stage_2_thread_0 & AM_iobusy_readAddr_0
  W329 := ~ io.readAddr_1.valid
  AM_iobusy_readAddr_1 := W329 & W328
  W330 := AM_perStageThreadSel_stage_2_thread_1 & AM_iobusy_readAddr_1
  W332 := ~ io.readAddr_2.valid
  AM_iobusy_readAddr_2 := W332 & W331
  W333 := AM_perStageThreadSel_stage_2_thread_2 & AM_iobusy_readAddr_2
  W335 := ~ io.readAddr_3.valid
  AM_iobusy_readAddr_3 := W335 & W334
  W336 := AM_perStageThreadSel_stage_2_thread_3 & AM_iobusy_readAddr_3
  AM_varLatIO_busy_imemPort_0 := io.imemPort_0.respPending & AM_perStageThreadSel_stage_0_thread_0
  AM_varLatIO_busy_imemPort_1 := io.imemPort_1.respPending & AM_perStageThreadSel_stage_0_thread_1
  AM_varLatIO_busy_imemPort_2 := io.imemPort_2.respPending & AM_perStageThreadSel_stage_0_thread_2
  AM_varLatIO_busy_imemPort_3 := io.imemPort_3.respPending & AM_perStageThreadSel_stage_0_thread_3
  AM_varLatIO_busy_dmemPort_0 := io.dmemPort_0.respPending & AM_perStageThreadSel_stage_3_thread_0
  AM_varLatIO_busy_dmemPort_1 := io.dmemPort_1.respPending & AM_perStageThreadSel_stage_3_thread_1
  AM_varLatIO_busy_dmemPort_2 := io.dmemPort_2.respPending & AM_perStageThreadSel_stage_3_thread_2
  AM_varLatIO_busy_dmemPort_3 := io.dmemPort_3.respPending & AM_perStageThreadSel_stage_3_thread_3
  W337 := Bool(true)
  W338 := ~ AM_RAW_hazard_2_pcReg_0_stage_3_writeNum_0
  W339 := W337 & W338
  W340 := ~ AM_RAW_hazard_1_pcReg_0_stage_2_writeNum_0
  W341 := W339 & W340
  W342 := ~ AM_RAW_hazard_5_pcReg_0_stage_3_writeNum_1
  W343 := W341 & W342
  W344 := ~ AM_RAW_hazard_4_pcReg_0_stage_2_writeNum_1
  W345 := W343 & W344
  W346 := ~ AM_RAW_hazard_0_pcReg_0_stage_1_writeNum_0
  W347 := W345 & W346
  W348 := ~ AM_RAW_hazard_3_pcReg_0_stage_1_writeNum_1
  W349 := W347 & W348
  AM_stage_NoRAW_0 := W349
  W350 := Bool(true)
  AM_stage_NoRAW_1 := W350
  W351 := Bool(true)
  W352 := ~ AM_RAW_hazard_9_regfile_0_readNum_0_stage_3_writeNum_1
  W353 := W351 & W352
  W354 := ~ AM_RAW_hazard_8_regfile_0_readNum_2_stage_3_writeNum_0
  W355 := W353 & W354
  W356 := ~ AM_RAW_hazard_7_regfile_0_readNum_1_stage_3_writeNum_0
  W357 := W355 & W356
  W358 := ~ AM_RAW_hazard_11_regfile_0_readNum_2_stage_3_writeNum_1
  W359 := W357 & W358
  W360 := ~ AM_RAW_hazard_10_regfile_0_readNum_1_stage_3_writeNum_1
  W361 := W359 & W360
  W362 := ~ AM_RAW_hazard_14_regfile_0_readNum_2_stage_3_writeNum_2
  W363 := W361 & W362
  W364 := ~ AM_RAW_hazard_13_regfile_0_readNum_1_stage_3_writeNum_2
  W365 := W363 & W364
  W366 := ~ AM_RAW_hazard_12_regfile_0_readNum_0_stage_3_writeNum_2
  W367 := W365 & W366
  W368 := ~ AM_RAW_hazard_6_regfile_0_readNum_0_stage_3_writeNum_0
  W369 := W367 & W368
  AM_stage_NoRAW_2 := W369
  W370 := Bool(true)
  AM_stage_NoRAW_3 := W370
  W371 := Bool(true)
  AM_stage_NoIOBusy_0 := W371
  W372 := Bool(true)
  AM_stage_NoIOBusy_1 := W372
  W373 := Bool(true)
  W374 := ~ W330
  W375 := W373 & W374
  W376 := ~ W327
  W377 := W375 & W376
  W378 := ~ W336
  W379 := W377 & W378
  W380 := ~ W333
  W381 := W379 & W380
  AM_stage_NoIOBusy_2 := W381
  W382 := Bool(true)
  W383 := ~ W321
  W384 := W382 & W383
  W385 := ~ W315
  W386 := W384 & W385
  W387 := ~ W324
  W388 := W386 & W387
  W389 := ~ W318
  W390 := W388 & W389
  AM_stage_NoIOBusy_3 := W390
  W391 := ~ AM_stage_kill_0
  W392 := W32 & W391
  W393 := AM_stage_NoRAW_0 & AM_stage_NoIOBusy_0
  W394 := W392 & W393
  AM_stage_valid_0 := W394
  W395 := ~ AM_stage_kill_1
  W396 := AM_prev_stage_valid_reg1 & W395
  W397 := AM_stage_NoRAW_1 & AM_stage_NoIOBusy_1
  W398 := W396 & W397
  AM_stage_valid_1 := W398
  W399 := ~ AM_stage_kill_2
  W400 := AM_prev_stage_valid_reg2 & W399
  W401 := AM_stage_NoRAW_2 & AM_stage_NoIOBusy_2
  W402 := W400 & W401
  AM_stage_valid_2 := W402
  W403 := ~ AM_stage_kill_3
  W404 := AM_prev_stage_valid_reg3 & W403
  W405 := AM_stage_NoRAW_3 & AM_stage_NoIOBusy_3
  W406 := W404 & W405
  AM_stage_valid_3 := W406
  W407 := Bool(false)
  AM_stage_stall_3 := W407
  W408 := ~ AM_stage_NoRAW_1
  W409 := ~ AM_stage_NoIOBusy_1
  W410 := W408 | W409
  W411 := AM_prev_stage_valid_reg1 & W410
  W412 := AM_stage_stall_1 | W411
  AM_stage_stall_0 := W412
  W413 := ~ AM_stage_NoRAW_2
  W414 := ~ AM_stage_NoIOBusy_2
  W415 := W413 | W414
  W416 := AM_prev_stage_valid_reg2 & W415
  W417 := AM_stage_stall_2 | W416
  AM_stage_stall_1 := W417
  W418 := ~ AM_stage_NoRAW_3
  W419 := ~ AM_stage_NoIOBusy_3
  W420 := W418 | W419
  W421 := AM_prev_stage_valid_reg3 & W420
  W422 := AM_stage_stall_3 | W421
  AM_stage_stall_2 := W422
  W423 := Bool(false)
  W424 := W423 | AM_varLatIO_busy_imemPort_0
  W425 := W424 | AM_varLatIO_busy_imemPort_1
  W426 := W425 | AM_varLatIO_busy_imemPort_2
  W427 := W426 | AM_varLatIO_busy_imemPort_3
  AM_stage_kill_0 := W427
  W428 := Bool(false)
  AM_stage_kill_1 := W428
  W429 := Bool(false)
  AM_stage_kill_2 := W429
  W430 := Bool(false)
  W431 := W430 | AM_varLatIO_busy_dmemPort_0
  W432 := W431 | AM_varLatIO_busy_dmemPort_1
  W433 := W432 | AM_varLatIO_busy_dmemPort_2
  W434 := W433 | AM_varLatIO_busy_dmemPort_3
  AM_stage_kill_3 := W434
  W435 := isNotJmp & AM_stage_valid_3
  W436 := isJmp & AM_stage_valid_3
  W437 := isNotJmp & AM_stage_valid_3
  W438 := isJmp & AM_stage_valid_3
  W439 := isNotJmp & AM_stage_valid_3
  W440 := isJmp & AM_stage_valid_3
  W441 := isNotJmp & AM_stage_valid_3
  W442 := isJmp & AM_stage_valid_3
  W443 := adderSel & AM_stage_valid_3
  W444 := subtractSel & AM_stage_valid_3
  W445 := AM_pipe_reg_0_stage_2 & AM_stage_valid_3
  W446 := adderSel & AM_stage_valid_3
  W447 := subtractSel & AM_stage_valid_3
  W448 := AM_pipe_reg_0_stage_2 & AM_stage_valid_3
  W449 := adderSel & AM_stage_valid_3
  W450 := subtractSel & AM_stage_valid_3
  W451 := AM_pipe_reg_0_stage_2 & AM_stage_valid_3
  W452 := adderSel & AM_stage_valid_3
  W453 := subtractSel & AM_stage_valid_3
  W454 := AM_pipe_reg_0_stage_2 & AM_stage_valid_3
  W455 := ~ AM_stage_kill_3
  W456 := AM_prev_stage_valid_reg3 & W455
  W457 := AM_stage_NoRAW_3 & W456
  W459 := W458 & W457
  W461 := W460 & W457
  W463 := W462 & W457
  W465 := W464 & W457
  W466 := ~ AM_stage_kill_2
  W467 := AM_prev_stage_valid_reg2 & W466
  W468 := AM_stage_NoRAW_2 & W467
  W470 := W469 & W468
  W472 := W471 & W468
  W474 := W473 & W468
  W476 := W475 & W468
  W477 := W32 & AM_stage_NoIOBusy_0
  W478 := AM_stage_NoRAW_0 & W477
  W480 := W479 & W478
  W482 := W481 & W478
  W484 := W483 & W478
  W486 := W485 & W478
  W487 := AM_prev_stage_valid_reg3 & AM_stage_NoIOBusy_3
  W488 := AM_stage_NoRAW_3 & W487
  W490 := W489 & W488
  W492 := W491 & W488
  W494 := W493 & W488
  W496 := W495 & W488
  W497 := W212 & AM_stage_valid_0
  W498 := W214 & AM_stage_valid_0
  W499 := ~ AM_stage_stall_3
  W500 := W435 & W499
  W501 := ~ AM_stage_stall_3
  W502 := W436 & W501
  W503 := ~ AM_stage_stall_3
  W504 := W437 & W503
  W505 := ~ AM_stage_stall_3
  W506 := W438 & W505
  W507 := ~ AM_stage_stall_3
  W508 := W439 & W507
  W509 := ~ AM_stage_stall_3
  W510 := W440 & W509
  W511 := ~ AM_stage_stall_3
  W512 := W441 & W511
  W513 := ~ AM_stage_stall_3
  W514 := W442 & W513
  W515 := ~ AM_stage_stall_3
  W516 := W443 & W515
  W517 := ~ AM_stage_stall_3
  W518 := W444 & W517
  W519 := ~ AM_stage_stall_3
  W520 := W445 & W519
  W521 := ~ AM_stage_stall_3
  W522 := W446 & W521
  W523 := ~ AM_stage_stall_3
  W524 := W447 & W523
  W525 := ~ AM_stage_stall_3
  W526 := W448 & W525
  W527 := ~ AM_stage_stall_3
  W528 := W449 & W527
  W529 := ~ AM_stage_stall_3
  W530 := W450 & W529
  W531 := ~ AM_stage_stall_3
  W532 := W451 & W531
  W533 := ~ AM_stage_stall_3
  W534 := W452 & W533
  W535 := ~ AM_stage_stall_3
  W536 := W453 & W535
  W537 := ~ AM_stage_stall_3
  W538 := W454 & W537
  W539 := ~ AM_stage_stall_3
  W541 := W540 & W539
  W542 := ~ AM_stage_stall_3
  W544 := W543 & W542
  W545 := ~ AM_stage_stall_3
  W547 := W546 & W545
  W548 := ~ AM_stage_stall_3
  W550 := W549 & W548
  W551 := ~ AM_stage_stall_2
  W553 := W552 & W551
  W554 := ~ AM_stage_stall_2
  W556 := W555 & W554
  W557 := ~ AM_stage_stall_2
  W559 := W558 & W557
  W560 := ~ AM_stage_stall_2
  W562 := W561 & W560
  W563 := ~ AM_stage_stall_0
  W565 := W564 & W563
  W566 := ~ AM_stage_stall_0
  W568 := W567 & W566
  W569 := ~ AM_stage_stall_0
  W571 := W570 & W569
  W572 := ~ AM_stage_stall_0
  W574 := W573 & W572
  W575 := ~ AM_stage_stall_3
  W577 := W576 & W575
  W578 := ~ AM_stage_stall_3
  W580 := W579 & W578
  W581 := ~ AM_stage_stall_3
  W583 := W582 & W581
  W584 := ~ AM_stage_stall_3
  W586 := W585 & W584
  W587 := ~ AM_stage_stall_0
  W588 := W48 & W587
  W589 := ~ AM_stage_stall_0
  W590 := W78 & W589
  W591 := ~ AM_stage_stall_1
  W592 := W49 & W591
  W593 := ~ AM_stage_stall_1
  W594 := W62 & W593
  W595 := ~ AM_stage_stall_1
  W596 := W66 & W595
  W597 := ~ AM_stage_stall_1
  W598 := W68 & W597
  W599 := ~ AM_stage_stall_1
  W600 := W70 & W599
  W601 := ~ AM_stage_stall_1
  W602 := W72 & W601
  W603 := ~ AM_stage_stall_1
  W604 := W74 & W603
  W605 := ~ AM_stage_stall_1
  W606 := W76 & W605
  W607 := ~ AM_stage_stall_1
  W608 := W82 & W607
  W609 := ~ AM_stage_stall_1
  W610 := W84 & W609
  W611 := ~ AM_stage_stall_1
  W612 := W96 & W611
  W613 := ~ AM_stage_stall_1
  W614 := W98 & W613
  W615 := ~ AM_stage_stall_1
  W616 := W100 & W615
  W617 := ~ AM_stage_stall_1
  W618 := W102 & W617
  W619 := ~ AM_stage_stall_1
  W620 := W104 & W619
  W621 := ~ AM_stage_stall_1
  W622 := W106 & W621
  W623 := ~ AM_stage_stall_1
  W624 := W108 & W623
  W625 := ~ AM_stage_stall_2
  W626 := W40 & W625
  W627 := ~ AM_stage_stall_2
  W628 := W42 & W627
  W629 := ~ AM_stage_stall_2
  W630 := W44 & W629
  W631 := ~ AM_stage_stall_2
  W632 := W46 & W631
  W633 := ~ AM_stage_stall_2
  W634 := W50 & W633
  W635 := ~ AM_stage_stall_2
  W636 := W52 & W635
  W637 := ~ AM_stage_stall_2
  W638 := W54 & W637
  W639 := ~ AM_stage_stall_2
  W640 := W56 & W639
  W641 := ~ AM_stage_stall_2
  W642 := W58 & W641
  W643 := ~ AM_stage_stall_2
  W644 := W60 & W643
  W645 := ~ AM_stage_stall_2
  W646 := W64 & W645
  W647 := ~ AM_stage_stall_2
  W648 := W80 & W647
  W649 := ~ AM_stage_stall_2
  W650 := W86 & W649
  W651 := ~ AM_stage_stall_2
  W652 := W88 & W651
  W653 := ~ AM_stage_stall_2
  W654 := W90 & W653
  W655 := ~ AM_stage_stall_2
  W656 := W92 & W655
  W657 := ~ AM_stage_stall_2
  W658 := W94 & W657
  W659 := ~ AM_stage_stall_0
  W660 := W33 & W659
  W661 := ~ AM_stage_stall_1
  W662 := W34 & W661
  W663 := ~ AM_stage_stall_2
  W664 := W35 & W663
  W665 := ~ AM_stage_stall_0
  W666 := W36 & W665
  W667 := ~ AM_stage_stall_1
  W668 := W37 & W667
  W669 := ~ AM_stage_stall_2
  W670 := W38 & W669
  W671 := ~ AM_stage_stall_0
  W672 := W497 & W671
  W673 := ~ AM_stage_stall_0
  W674 := W498 & W673
  AM_regWEn_pcReg_0_thread_0_writeNum_0 := W500 & AM_perStageThreadSel_stage_3_thread_0
  AM_regWEn_pcReg_0_thread_0_writeNum_1 := W502 & AM_perStageThreadSel_stage_3_thread_0
  AM_regWEn_pcReg_0_thread_1_writeNum_0 := W504 & AM_perStageThreadSel_stage_3_thread_1
  AM_regWEn_pcReg_0_thread_1_writeNum_1 := W506 & AM_perStageThreadSel_stage_3_thread_1
  AM_regWEn_pcReg_0_thread_2_writeNum_0 := W508 & AM_perStageThreadSel_stage_3_thread_2
  AM_regWEn_pcReg_0_thread_2_writeNum_1 := W510 & AM_perStageThreadSel_stage_3_thread_2
  AM_regWEn_pcReg_0_thread_3_writeNum_0 := W512 & AM_perStageThreadSel_stage_3_thread_3
  AM_regWEn_pcReg_0_thread_3_writeNum_1 := W514 & AM_perStageThreadSel_stage_3_thread_3
  AM_memWEn_regfile_0_thread_0_writeNum_0 := W516 & AM_perStageThreadSel_stage_3_thread_0
  AM_memWEn_regfile_0_thread_0_writeNum_1 := W518 & AM_perStageThreadSel_stage_3_thread_0
  AM_memWEn_regfile_0_thread_0_writeNum_2 := W520 & AM_perStageThreadSel_stage_3_thread_0
  AM_memWEn_regfile_0_thread_1_writeNum_0 := W522 & AM_perStageThreadSel_stage_3_thread_1
  AM_memWEn_regfile_0_thread_1_writeNum_1 := W524 & AM_perStageThreadSel_stage_3_thread_1
  AM_memWEn_regfile_0_thread_1_writeNum_2 := W526 & AM_perStageThreadSel_stage_3_thread_1
  AM_memWEn_regfile_0_thread_2_writeNum_0 := W528 & AM_perStageThreadSel_stage_3_thread_2
  AM_memWEn_regfile_0_thread_2_writeNum_1 := W530 & AM_perStageThreadSel_stage_3_thread_2
  AM_memWEn_regfile_0_thread_2_writeNum_2 := W532 & AM_perStageThreadSel_stage_3_thread_2
  AM_memWEn_regfile_0_thread_3_writeNum_0 := W534 & AM_perStageThreadSel_stage_3_thread_3
  AM_memWEn_regfile_0_thread_3_writeNum_1 := W536 & AM_perStageThreadSel_stage_3_thread_3
  AM_memWEn_regfile_0_thread_3_writeNum_2 := W538 & AM_perStageThreadSel_stage_3_thread_3
  W676 := W675 & AM_perStageThreadSel_stage_3_thread_0
  W678 := W677 & AM_perStageThreadSel_stage_3_thread_1
  W680 := W679 & AM_perStageThreadSel_stage_3_thread_2
  W682 := W681 & AM_perStageThreadSel_stage_3_thread_3
  W684 := W683 & AM_perStageThreadSel_stage_2_thread_0
  W686 := W685 & AM_perStageThreadSel_stage_2_thread_1
  W688 := W687 & AM_perStageThreadSel_stage_2_thread_2
  W690 := W689 & AM_perStageThreadSel_stage_2_thread_3
  W692 := W691 & AM_perStageThreadSel_stage_0_thread_0
  W694 := W693 & AM_perStageThreadSel_stage_0_thread_1
  W696 := W695 & AM_perStageThreadSel_stage_0_thread_2
  W698 := W697 & AM_perStageThreadSel_stage_0_thread_3
  W700 := W699 & AM_perStageThreadSel_stage_3_thread_0
  W702 := W701 & AM_perStageThreadSel_stage_3_thread_1
  W704 := W703 & AM_perStageThreadSel_stage_3_thread_2
  W706 := W705 & AM_perStageThreadSel_stage_3_thread_3
  io.readData_0.valid := W676
  io.readData_0.bits := W110
  io.readAddr_0.ready := W684
  io.imemPort_0.reqValid := W692
  io.imemPort_0.reqBits := W117
  io.dmemPort_0.reqValid := W700
  io.dmemPort_0.reqBits := W123
  AM_regWData_pcReg_0_thread_3_writeNum_0 := AM_pipe_reg_6_stage_2pcPlus4
  inst := AM_pipe_reg_20_stage_0inst
  AM_memWAddr_regfile_0_thread_3_writeNum_2 := AM_pipe_reg_9_stage_2rd
  imm := AM_pipe_reg_19_stage_1imm
  AM_regWData_pcReg_0_thread_3_writeNum_1 := AM_pipe_reg_8_stage_2jmpTarget
  isJmp := AM_pipe_reg_7_stage_2isJmp
  isNotJmp := AM_pipe_reg_3_stage_2isNotJmp
  W15 := AM_pipe_reg_31_stage_1W15
  W17 := AM_pipe_reg_32_stage_1W17
  adderSel := AM_pipe_reg_10_stage_2adderSel
  subtractSel := AM_pipe_reg_11_stage_2subtractSel
  isLoad := AM_pipe_reg_12_stage_1isLoad
  isStore := AM_pipe_reg_33_stage_1isStore
  W30 := AM_pipe_reg_21_stage_2W30
  W39 := isLoad
  W65 := rs1
  W67 := rs2
  W69 := op
  W71 := op
  W73 := inst
  W81 := op
  W83 := op
  W85 := W10
  W87 := AM_mem_read_data_mux_regfile_0_read_port_num_0
  W89 := W13
  W91 := imm
  W93 := AM_mem_read_data_mux_regfile_0_read_port_num_1
  W95 := op
  W97 := op
  W105 := op
  W107 := op
  io.readData_1.valid := W678
  io.readData_1.bits := W110
  io.readData_2.valid := W680
  io.readData_2.bits := W110
  io.readData_3.valid := W682
  io.readData_3.bits := W110
  io.readAddr_1.ready := W686
  io.readAddr_2.ready := W688
  io.readAddr_3.ready := W690
  io.imemPort_1.reqValid := W694
  io.imemPort_1.reqBits := W113
  io.imemPort_2.reqValid := W696
  io.imemPort_2.reqBits := W115
  io.imemPort_3.reqValid := W698
  io.imemPort_3.reqBits := W117
  io.dmemPort_1.reqValid := W702
  io.dmemPort_1.reqBits := W119
  io.dmemPort_2.reqValid := W704
  io.dmemPort_2.reqBits := W121
  io.dmemPort_3.reqValid := W706
  io.dmemPort_3.reqBits := W123
  W109 := AM_pipe_reg_1_stage_2valid
  W110 := AM_pipe_reg_2_stage_2bits
  W114 := W112
  W115 := W113
  W116 := W114
  W117 := W115
  W118 := AM_pipe_reg_13_stage_2reqValid
  W120 := W118
  W121 := W119
  W122 := W120
  W123 := W121
  W232 := W45
  W235 := W45
  W238 := isNotJmp
  W241 := W51
  W244 := W51
  W247 := isJmp
  W250 := adderSel
  W251 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W257 := adderSel
  W258 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W264 := adderSel
  W265 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W271 := subtractSel
  W272 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W278 := subtractSel
  W279 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W285 := subtractSel
  W286 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W292 := AM_pipe_reg_0_stage_2
  W293 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W299 := AM_pipe_reg_0_stage_2
  W300 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W306 := AM_pipe_reg_0_stage_2
  W307 := AM_memWAddr_regfile_0_thread_3_writeNum_2
  W313 := W109
  W316 := W109
  W319 := W109
  W322 := W109
  W325 := W111
  W328 := W111
  W331 := W111
  W334 := W111
  W458 := W313
  W460 := W316
  W462 := W319
  W464 := W322
  W469 := W325
  W471 := W328
  W473 := W331
  W475 := W334
  W479 := W116
  W481 := W112
  W483 := W114
  W485 := W116
  W489 := W122
  W491 := W118
  W493 := W120
  W495 := W122
  W540 := W459
  W543 := W461
  W546 := W463
  W549 := W465
  W552 := W470
  W555 := W472
  W558 := W474
  W561 := W476
  W564 := W480
  W567 := W482
  W570 := W484
  W573 := W486
  W576 := W490
  W579 := W492
  W582 := W494
  W585 := W496
  W675 := W541
  W677 := W544
  W679 := W547
  W681 := W550
  W683 := W553
  W685 := W556
  W687 := W559
  W689 := W562
  W691 := W565
  W693 := W568
  W695 := W571
  W697 := W574
  W699 := W577
  W701 := W580
  W703 := W583
  W705 := W586
  val pcReg_0_reg = Reg(init = Bits(0, width = 4))
  pcReg_0 := pcReg_0_reg
  when(AM_regWEn_pcReg_0_thread_0_writeNum_0){
    pcReg_0_reg := AM_regWData_pcReg_0_thread_3_writeNum_0
  }
  when(AM_regWEn_pcReg_0_thread_0_writeNum_1){
    pcReg_0_reg := AM_regWData_pcReg_0_thread_3_writeNum_1
  }
  val regfile_0 = Mem(Bits(width = 32), 16)
  rs1Data := regfile_0.read(AM_pipe_reg_14_stage_1)
  rs2Data := regfile_0.read(AM_pipe_reg_15_stage_1)
  W43 := regfile_0.read(AM_inputIO_bits_mux_readAddr_0)
  when(AM_memWEn_regfile_0_thread_0_writeNum_0){
    regfile_0.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_0)
  }
  when(AM_memWEn_regfile_0_thread_0_writeNum_1){
    regfile_0.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_1)
  }
  when(AM_memWEn_regfile_0_thread_0_writeNum_2){
    regfile_0.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_2)
  }
  val AM_prev_stage_valid_reg1_reg = Reg(init = Bool(false))
  AM_prev_stage_valid_reg1 := AM_prev_stage_valid_reg1_reg
  when(W660){
    AM_prev_stage_valid_reg1_reg := AM_stage_valid_0
  }
  val AM_prev_stage_valid_reg2_reg = Reg(init = Bool(false))
  AM_prev_stage_valid_reg2 := AM_prev_stage_valid_reg2_reg
  when(W662){
    AM_prev_stage_valid_reg2_reg := AM_stage_valid_1
  }
  val AM_prev_stage_valid_reg3_reg = Reg(init = Bool(false))
  AM_prev_stage_valid_reg3 := AM_prev_stage_valid_reg3_reg
  when(W664){
    AM_prev_stage_valid_reg3_reg := AM_stage_valid_2
  }
  val AM_stage_thread_sel_id_1_reg = Reg(init = Bits(0, width = 3))
  AM_stage_thread_sel_id_1 := AM_stage_thread_sel_id_1_reg
  when(W666){
    AM_stage_thread_sel_id_1_reg := AM_stage_thread_sel_id_0
  }
  val AM_stage_thread_sel_id_2_reg = Reg(init = Bits(0, width = 3))
  AM_stage_thread_sel_id_2 := AM_stage_thread_sel_id_2_reg
  when(W668){
    AM_stage_thread_sel_id_2_reg := AM_stage_thread_sel_id_1
  }
  val AM_stage_thread_sel_id_3_reg = Reg(init = Bits(0, width = 3))
  AM_stage_thread_sel_id_3 := AM_stage_thread_sel_id_3_reg
  when(W670){
    AM_stage_thread_sel_id_3_reg := AM_stage_thread_sel_id_2
  }
  val AM_pipe_reg_0_stage_2_reg = Reg(init = Bool(false))
  AM_pipe_reg_0_stage_2 := AM_pipe_reg_0_stage_2_reg
  when(W626){
    AM_pipe_reg_0_stage_2_reg := W39
  }
  val AM_pipe_reg_1_stage_2valid_reg = Reg(init = Bool(false))
  AM_pipe_reg_1_stage_2valid := AM_pipe_reg_1_stage_2valid_reg
  when(W628){
    AM_pipe_reg_1_stage_2valid_reg := W41
  }
  val AM_pipe_reg_2_stage_2bits_reg = Reg(init = Bits(1))
  AM_pipe_reg_2_stage_2bits := AM_pipe_reg_2_stage_2bits_reg
  when(W630){
    AM_pipe_reg_2_stage_2bits_reg := AM_mem_read_data_mux_regfile_0_read_port_num_2
  }
  val AM_pipe_reg_3_stage_2isNotJmp_reg = Reg(init = Bool(false))
  AM_pipe_reg_3_stage_2isNotJmp := AM_pipe_reg_3_stage_2isNotJmp_reg
  when(W632){
    AM_pipe_reg_3_stage_2isNotJmp_reg := W45
  }
  val AM_pipe_reg_4_stage_0pcPlus4_reg = Reg(init = Bits(1))
  AM_pipe_reg_4_stage_0pcPlus4 := AM_pipe_reg_4_stage_0pcPlus4_reg
  when(W588){
    AM_pipe_reg_4_stage_0pcPlus4_reg := W47
  }
  val AM_pipe_reg_5_stage_1pcPlus4_reg = Reg(init = Bits(1))
  AM_pipe_reg_5_stage_1pcPlus4 := AM_pipe_reg_5_stage_1pcPlus4_reg
  when(W592){
    AM_pipe_reg_5_stage_1pcPlus4_reg := AM_pipe_reg_4_stage_0pcPlus4
  }
  val AM_pipe_reg_6_stage_2pcPlus4_reg = Reg(init = Bits(1))
  AM_pipe_reg_6_stage_2pcPlus4 := AM_pipe_reg_6_stage_2pcPlus4_reg
  when(W634){
    AM_pipe_reg_6_stage_2pcPlus4_reg := AM_pipe_reg_5_stage_1pcPlus4
  }
  val AM_pipe_reg_7_stage_2isJmp_reg = Reg(init = Bool(false))
  AM_pipe_reg_7_stage_2isJmp := AM_pipe_reg_7_stage_2isJmp_reg
  when(W636){
    AM_pipe_reg_7_stage_2isJmp_reg := W51
  }
  val AM_pipe_reg_8_stage_2jmpTarget_reg = Reg(init = Bits(1))
  AM_pipe_reg_8_stage_2jmpTarget := AM_pipe_reg_8_stage_2jmpTarget_reg
  when(W638){
    AM_pipe_reg_8_stage_2jmpTarget_reg := W53
  }
  val AM_pipe_reg_9_stage_2rd_reg = Reg(init = Bits(1))
  AM_pipe_reg_9_stage_2rd := AM_pipe_reg_9_stage_2rd_reg
  when(W640){
    AM_pipe_reg_9_stage_2rd_reg := W55
  }
  val AM_pipe_reg_10_stage_2adderSel_reg = Reg(init = Bool(false))
  AM_pipe_reg_10_stage_2adderSel := AM_pipe_reg_10_stage_2adderSel_reg
  when(W642){
    AM_pipe_reg_10_stage_2adderSel_reg := W57
  }
  val AM_pipe_reg_11_stage_2subtractSel_reg = Reg(init = Bool(false))
  AM_pipe_reg_11_stage_2subtractSel := AM_pipe_reg_11_stage_2subtractSel_reg
  when(W644){
    AM_pipe_reg_11_stage_2subtractSel_reg := W59
  }
  val AM_pipe_reg_12_stage_1isLoad_reg = Reg(init = Bool(false))
  AM_pipe_reg_12_stage_1isLoad := AM_pipe_reg_12_stage_1isLoad_reg
  when(W594){
    AM_pipe_reg_12_stage_1isLoad_reg := W61
  }
  val AM_pipe_reg_13_stage_2reqValid_reg = Reg(init = Bool(false))
  AM_pipe_reg_13_stage_2reqValid := AM_pipe_reg_13_stage_2reqValid_reg
  when(W646){
    AM_pipe_reg_13_stage_2reqValid_reg := W63
  }
  val AM_pipe_reg_14_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_14_stage_1 := AM_pipe_reg_14_stage_1_reg
  when(W596){
    AM_pipe_reg_14_stage_1_reg := W65
  }
  val AM_pipe_reg_15_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_15_stage_1 := AM_pipe_reg_15_stage_1_reg
  when(W598){
    AM_pipe_reg_15_stage_1_reg := W67
  }
  val AM_pipe_reg_16_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_16_stage_1 := AM_pipe_reg_16_stage_1_reg
  when(W600){
    AM_pipe_reg_16_stage_1_reg := W69
  }
  val AM_pipe_reg_17_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_17_stage_1 := AM_pipe_reg_17_stage_1_reg
  when(W602){
    AM_pipe_reg_17_stage_1_reg := W71
  }
  val AM_pipe_reg_18_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_18_stage_1 := AM_pipe_reg_18_stage_1_reg
  when(W604){
    AM_pipe_reg_18_stage_1_reg := W73
  }
  val AM_pipe_reg_19_stage_1imm_reg = Reg(init = Bits(1))
  AM_pipe_reg_19_stage_1imm := AM_pipe_reg_19_stage_1imm_reg
  when(W606){
    AM_pipe_reg_19_stage_1imm_reg := W75
  }
  val AM_pipe_reg_20_stage_0inst_reg = Reg(init = Bits(1))
  AM_pipe_reg_20_stage_0inst := AM_pipe_reg_20_stage_0inst_reg
  when(W590){
    AM_pipe_reg_20_stage_0inst_reg := W77
  }
  val AM_pipe_reg_21_stage_2W30_reg = Reg(init = Bits(1))
  AM_pipe_reg_21_stage_2W30 := AM_pipe_reg_21_stage_2W30_reg
  when(W648){
    AM_pipe_reg_21_stage_2W30_reg := W79
  }
  val AM_pipe_reg_22_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_22_stage_1 := AM_pipe_reg_22_stage_1_reg
  when(W608){
    AM_pipe_reg_22_stage_1_reg := W81
  }
  val AM_pipe_reg_23_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_23_stage_1 := AM_pipe_reg_23_stage_1_reg
  when(W610){
    AM_pipe_reg_23_stage_1_reg := W83
  }
  val AM_pipe_reg_24_stage_2_reg = Reg(init = Bool(false))
  AM_pipe_reg_24_stage_2 := AM_pipe_reg_24_stage_2_reg
  when(W650){
    AM_pipe_reg_24_stage_2_reg := W85
  }
  val AM_pipe_reg_25_stage_2_reg = Reg(init = Bits(1))
  AM_pipe_reg_25_stage_2 := AM_pipe_reg_25_stage_2_reg
  when(W652){
    AM_pipe_reg_25_stage_2_reg := W87
  }
  val AM_pipe_reg_26_stage_2_reg = Reg(init = Bool(false))
  AM_pipe_reg_26_stage_2 := AM_pipe_reg_26_stage_2_reg
  when(W654){
    AM_pipe_reg_26_stage_2_reg := W89
  }
  val AM_pipe_reg_27_stage_2_reg = Reg(init = Bits(1))
  AM_pipe_reg_27_stage_2 := AM_pipe_reg_27_stage_2_reg
  when(W656){
    AM_pipe_reg_27_stage_2_reg := W91
  }
  val AM_pipe_reg_28_stage_2_reg = Reg(init = Bits(1))
  AM_pipe_reg_28_stage_2 := AM_pipe_reg_28_stage_2_reg
  when(W658){
    AM_pipe_reg_28_stage_2_reg := W93
  }
  val AM_pipe_reg_29_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_29_stage_1 := AM_pipe_reg_29_stage_1_reg
  when(W612){
    AM_pipe_reg_29_stage_1_reg := W95
  }
  val AM_pipe_reg_30_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_30_stage_1 := AM_pipe_reg_30_stage_1_reg
  when(W614){
    AM_pipe_reg_30_stage_1_reg := W97
  }
  val AM_pipe_reg_31_stage_1W15_reg = Reg(init = Bool(false))
  AM_pipe_reg_31_stage_1W15 := AM_pipe_reg_31_stage_1W15_reg
  when(W616){
    AM_pipe_reg_31_stage_1W15_reg := W99
  }
  val AM_pipe_reg_32_stage_1W17_reg = Reg(init = Bool(false))
  AM_pipe_reg_32_stage_1W17 := AM_pipe_reg_32_stage_1W17_reg
  when(W618){
    AM_pipe_reg_32_stage_1W17_reg := W101
  }
  val AM_pipe_reg_33_stage_1isStore_reg = Reg(init = Bool(false))
  AM_pipe_reg_33_stage_1isStore := AM_pipe_reg_33_stage_1isStore_reg
  when(W620){
    AM_pipe_reg_33_stage_1isStore_reg := W103
  }
  val AM_pipe_reg_34_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_34_stage_1 := AM_pipe_reg_34_stage_1_reg
  when(W622){
    AM_pipe_reg_34_stage_1_reg := W105
  }
  val AM_pipe_reg_35_stage_1_reg = Reg(init = Bits(1))
  AM_pipe_reg_35_stage_1 := AM_pipe_reg_35_stage_1_reg
  when(W624){
    AM_pipe_reg_35_stage_1_reg := W107
  }
  val pcReg_1_reg = Reg(init = Bits(0, width = 4))
  pcReg_1 := pcReg_1_reg
  when(AM_regWEn_pcReg_0_thread_1_writeNum_0){
    pcReg_1_reg := AM_regWData_pcReg_0_thread_3_writeNum_0
  }
  when(AM_regWEn_pcReg_0_thread_1_writeNum_1){
    pcReg_1_reg := AM_regWData_pcReg_0_thread_3_writeNum_1
  }
  val pcReg_2_reg = Reg(init = Bits(0, width = 4))
  pcReg_2 := pcReg_2_reg
  when(AM_regWEn_pcReg_0_thread_2_writeNum_0){
    pcReg_2_reg := AM_regWData_pcReg_0_thread_3_writeNum_0
  }
  when(AM_regWEn_pcReg_0_thread_2_writeNum_1){
    pcReg_2_reg := AM_regWData_pcReg_0_thread_3_writeNum_1
  }
  val pcReg_3_reg = Reg(init = Bits(0, width = 4))
  pcReg_3 := pcReg_3_reg
  when(AM_regWEn_pcReg_0_thread_3_writeNum_0){
    pcReg_3_reg := AM_regWData_pcReg_0_thread_3_writeNum_0
  }
  when(AM_regWEn_pcReg_0_thread_3_writeNum_1){
    pcReg_3_reg := AM_regWData_pcReg_0_thread_3_writeNum_1
  }
  val regfile_1 = Mem(Bits(width = 32), 16)
  W157 := regfile_1.read(AM_pipe_reg_14_stage_1)
  W158 := regfile_1.read(AM_pipe_reg_15_stage_1)
  W159 := regfile_1.read(AM_inputIO_bits_mux_readAddr_0)
  when(AM_memWEn_regfile_0_thread_1_writeNum_0){
    regfile_1.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_0)
  }
  when(AM_memWEn_regfile_0_thread_1_writeNum_1){
    regfile_1.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_1)
  }
  when(AM_memWEn_regfile_0_thread_1_writeNum_2){
    regfile_1.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_2)
  }
  val regfile_2 = Mem(Bits(width = 32), 16)
  W160 := regfile_2.read(AM_pipe_reg_14_stage_1)
  W161 := regfile_2.read(AM_pipe_reg_15_stage_1)
  W162 := regfile_2.read(AM_inputIO_bits_mux_readAddr_0)
  when(AM_memWEn_regfile_0_thread_2_writeNum_0){
    regfile_2.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_0)
  }
  when(AM_memWEn_regfile_0_thread_2_writeNum_1){
    regfile_2.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_1)
  }
  when(AM_memWEn_regfile_0_thread_2_writeNum_2){
    regfile_2.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_2)
  }
  val regfile_3 = Mem(Bits(width = 32), 16)
  W163 := regfile_3.read(AM_pipe_reg_14_stage_1)
  W164 := regfile_3.read(AM_pipe_reg_15_stage_1)
  W165 := regfile_3.read(AM_inputIO_bits_mux_readAddr_0)
  when(AM_memWEn_regfile_0_thread_3_writeNum_0){
    regfile_3.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_0)
  }
  when(AM_memWEn_regfile_0_thread_3_writeNum_1){
    regfile_3.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_1)
  }
  when(AM_memWEn_regfile_0_thread_3_writeNum_2){
    regfile_3.write(AM_memWAddr_regfile_0_thread_3_writeNum_2, AM_memWData_regfile_0_thread_3_writeNum_2)
  }
  val AM_thread_sel_counter_reg = Reg(init = Bits(0, width = 3))
  AM_thread_sel_counter := AM_thread_sel_counter_reg
  when(W672){
    AM_thread_sel_counter_reg := W211
  }
  when(W674){
    AM_thread_sel_counter_reg := W215
  }
}
