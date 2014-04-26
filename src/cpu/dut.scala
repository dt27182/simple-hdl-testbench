package Cpu
import Chisel._
import Common._
 
class Cpu extends Module {
  val io = new Bundle {
    var readData = new DecoupledIO(Bits(width = 32))
    var readAddr = new DecoupledIO(Bits(width = 4)).flip
    var imemPort = new VarLatIO(4, 32)
    var dmemPort = new VarLatIO(43, 32)
  }
  val pcReg = Bits()
  val W0 = Bits()
  val pcSpec = Bits()
  val W1 = Bits()
  val pcPlus4 = Bits()
  val W2 = Bool()
  val W3 = Bool()
  val inst = Bits()
  val rs1 = Bits()
  val rs2 = Bits()
  val rd = Bits()
  val op = Bits()
  val imm = Bits()
  val jmpTarget = Bits()
  val W4 = Bits()
  val isJmp = Bool()
  val W5 = Bits()
  val isNotJmp = Bool()
  val rs1Data = Bits()
  val rs2Data = Bits()
  val W6 = Bits()
  val isExternalRead = Bool()
  val W7 = Bits()
  val W8 = Bool()
  val W9 = Bits()
  val operand1 = Bits()
  val W10 = Bits()
  val W11 = Bool()
  val operand2 = Bits()
  val adderOut = Bits()
  val subtractOut = Bits()
  val W12 = Bits()
  val W13 = Bool()
  val W14 = Bits()
  val W15 = Bool()
  val W16 = Bool()
  val W17 = Bits()
  val W18 = Bool()
  val adderSel = Bool()
  val W19 = Bits()
  val W20 = Bool()
  val W21 = Bits()
  val W22 = Bool()
  val subtractSel = Bool()
  val W23 = Bits()
  val isLoad = Bool()
  val W24 = Bits()
  val isStore = Bool()
  val W25 = Bits()
  val W26 = Bits()
  val memWrite = Bits()
  val W27 = Bits()
  val W28 = Bits()
  val W29 = Bool()
  W0 := Bits(1, width = 4)
  pcSpec := pcReg + W0
  W1 := Bits(1, width = 4)
  pcPlus4 := pcReg + W1
  W2 := Bool(true)
  io.imemPort.req.valid := W2
  io.imemPort.req.bits := pcReg
  W3 := Bool(true)
  io.imemPort.resp.ready := W3
  inst := io.imemPort.resp.bits
  rs1 := inst(11, 8)
  rs2 := inst(7, 4)
  rd := inst(3, 0)
  op := inst(15, 12)
  imm := inst(31, 16)
  jmpTarget := imm(3, 0)
  W4 := Bits(6, width = 4)
  isJmp := op === W4
  W5 := Bits(6, width = 4)
  isNotJmp := op != W5
  W6 := Bits(7, width = 4)
  isExternalRead := op === W6
  io.readAddr.ready := isExternalRead
  io.readData.valid := isExternalRead
  W7 := Bits(4, width = 4)
  W8 := op === W7
  W9 := Bits(0, width = 32)
  operand1 := Mux(W8, W9, rs1Data)
  W10 := Bits(1, width = 4)
  W11 := op > W10
  operand2 := Mux(W11, imm, rs2Data)
  adderOut := operand1 + operand2
  subtractOut := operand1 - operand2
  W12 := Bits(0, width = 4)
  W13 := op === W12
  W14 := Bits(2, width = 4)
  W15 := op === W14
  W16 := W13 | W15
  W17 := Bits(4, width = 4)
  W18 := op === W17
  adderSel := W16 | W18
  W19 := Bits(1, width = 4)
  W20 := op === W19
  W21 := Bits(3, width = 4)
  W22 := op === W21
  subtractSel := W20 | W22
  W23 := Bits(8, width = 4)
  isLoad := op === W23
  W24 := Bits(9, width = 4)
  isStore := op === W24
  W25 := Bits(1, width = 1)
  W26 := Bits(0, width = 1)
  memWrite := Mux(isStore, W25, W26)
  W27 := rs1Data(9, 0)
  W28 := Cat(memWrite, W27, rs2Data)
  io.dmemPort.req.bits := W28
  W29 := isLoad | isStore
  io.dmemPort.req.valid := W29
  io.dmemPort.resp.ready := isLoad
  val pcReg_reg = Reg(init = Bits(0, width = 4))
  pcReg := pcReg_reg
  when(isNotJmp){
    pcReg_reg := pcPlus4
  }
  when(isJmp){
    pcReg_reg := jmpTarget
  }
  val regfile = Mem(Bits(width = 32), 16)
  rs1Data := regfile.read(rs1)
  rs2Data := regfile.read(rs2)
  io.readData.bits := regfile.read(io.readAddr.bits)
  when(adderSel){
    regfile.write(rd, adderOut)
  }
  when(subtractSel){
    regfile.write(rd, subtractOut)
  }
  when(isLoad){
    regfile.write(rd, io.dmemPort.resp.bits)
  }
}
