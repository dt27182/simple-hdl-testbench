package FSM
import Chisel._
import Common._
 
class TransactionMemDUT extends Module {
  val io = new Bundle {
    var addr_in = new DecoupledIO(Bits(width = 2)).flip
    var data_out = new DecoupledIO(Bits(width = 32))
    var icache = new VarLatIO(32, 32)
  }
  val write_data = Bits()
  val write_addr = Bits(width=2)
  val write_en = Bool()
  val memReadData = Bits()
  val currentState = Bits()
  val readData = Bits()
  val write0 = Bits()
  val write1 = Bits()
  val write2 = Bits()
  val write3 = Bits()
  val waitForRead = Bits()
  val sendReadData = Bits()
  val stateEqWrite0 = Bool()
  val stateEqWrite1 = Bool()
  val stateEqWrite2 = Bool()
  val stateEqWrite3 = Bool()
  val stateEqWaitForRead = Bool()
  val stateEqSendReadData = Bool()
  val W0 = Bool()
  val W1 = Bool()
  val W2 = Bool()
  val W3 = Bits()
  val W4 = Bits()
  val W5 = Bool()
  val W6 = Bool()
  val W7 = Bool()
  val W8 = Bits()
  val W9 = Bits()
  val W10 = Bits()
  val W11 = Bits()
  val W12 = Bits()
  val W13 = Bits()
  val W14 = Bits()
  val W15 = Bits()
  val W16 = Bits()
  val W17 = Bits()
  val W18 = Bits()
  val W19 = Bits()
  val W20 = Bits()
  val W21 = Bits()
  val W22 = Bits()
  val W23 = Bits()
  val W24 = Bits()
  val W25 = Bits()
  val W26 = Bool()
  val W27 = Bool()
  val W28 = Bool()
  val W29 = Bool()
  val W30 = Bool()
  val W31 = Bool()
  val W32 = Bool()
  val W33 = Bool()
  val W34 = Bool()
  val W35 = Bool()
  val W36 = Bool()
  val W37 = Bool()
  write0 := Bits(0, width = 4)
  write1 := Bits(1, width = 4)
  write2 := Bits(2, width = 4)
  write3 := Bits(3, width = 4)
  waitForRead := Bits(4, width = 4)
  sendReadData := Bits(5, width = 4)
  stateEqWrite0 := currentState === write0
  stateEqWrite1 := currentState === write1
  stateEqWrite2 := currentState === write2
  stateEqWrite3 := currentState === write3
  stateEqWaitForRead := currentState === waitForRead
  stateEqSendReadData := currentState === sendReadData
  W0 := Bool(true)
  W1 := Bool(false)
  W2 := Mux(stateEqWaitForRead, W0, W1)
  io.addr_in.ready := W2
  W3 := Bits(0, width = 32)
  W4 := Mux(stateEqSendReadData, readData, W3)
  io.data_out.bits := W4
  W5 := Bool(true)
  W6 := Bool(false)
  W7 := Mux(stateEqSendReadData, W5, W6)
  io.data_out.valid := W7
  W8 := Bits(3, width = 32)
  W9 := Bits(2, width = 32)
  W10 := Bits(1, width = 32)
  W11 := Bits(0, width = 32)
  W12 := Bits(0, width = 32)
  W13 := Mux(stateEqWrite0, W8, W12)
  W14 := Mux(stateEqWrite1, W9, W13)
  W15 := Mux(stateEqWrite2, W10, W14)
  W16 := Mux(stateEqWrite3, W11, W15)
  write_data := W16
  W17 := Bits(0, width = 2)
  W18 := Bits(1, width = 2)
  W19 := Bits(2, width = 2)
  W20 := Bits(3, width = 2)
  W21 := Bits(0, width = 2)
  W22 := Mux(stateEqWrite0, W17, W21)
  W23 := Mux(stateEqWrite1, W18, W22)
  W24 := Mux(stateEqWrite2, W19, W23)
  W25 := Mux(stateEqWrite3, W20, W24)
  write_addr := W25
  W26 := Bool(true)
  W27 := Bool(true)
  W28 := Bool(true)
  W29 := Bool(true)
  W30 := Bool(false)
  W31 := Mux(stateEqWrite0, W26, W30)
  W32 := Mux(stateEqWrite1, W27, W31)
  W33 := Mux(stateEqWrite2, W28, W32)
  W34 := Mux(stateEqWrite3, W29, W33)
  write_en := W34
  W35 := stateEqWaitForRead & io.addr_in.valid
  W36 := stateEqSendReadData & io.data_out.ready
  W37 := stateEqWaitForRead & io.addr_in.valid
  val mem = Mem(Bits(width = 32), 4)
  memReadData := mem.read(io.addr_in.bits)
  when(write_en){
    mem.write(write_addr, write_data)
  }
  val currentState_reg = Reg(init = Bits(0, width = 4))
  currentState := currentState_reg
  when(stateEqWrite0){
    currentState_reg := write1
  }
  when(stateEqWrite1){
    currentState_reg := write2
  }
  when(stateEqWrite2){
    currentState_reg := write3
  }
  when(stateEqWrite3){
    currentState_reg := waitForRead
  }
  when(W35){
    currentState_reg := sendReadData
  }
  when(W36){
    currentState_reg := waitForRead
  }
  val readData_reg = Reg(init = Bits(0, width = 32))
  readData := readData_reg
  when(W37){
    readData_reg := memReadData
  }
}
