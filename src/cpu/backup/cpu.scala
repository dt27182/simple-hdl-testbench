package Cpu 
 
import Chisel._ 
import scala.collection.mutable.HashMap
import Common._


class Cpu extends Module {
  val io = new Bundle { 
    val readData = new DecoupledIO(UInt(width = 32))
    val readAddr = new DecoupledIO(Bits(width = 4)).flip()
    val imemPort = new VarLatIO(4, 32)
    val dmemPort = new VarLatIO(43,32)
  }

  val pcReg = Reg(init = UInt(0,4))
  val pcSpec = UInt()
  pcSpec := pcReg + UInt(1)
  val pcPlus4 = UInt()
  pcPlus4 := pcReg + UInt(1)
  
  io.imemPort.req.valid := Bool(true)
  io.imemPort.req.bits := pcReg
  
  io.imemPort.resp.ready := Bool(true)
  val inst = Bits()
  inst := io.imemPort.resp.bits

  val rs1 = Bits()
  val rs2 = Bits()
  val rd = Bits()
  val op = Bits()
  val imm = Bits()
  rs1 := inst(11,8)
  rs2 := inst(7,4)
  rd := inst(3,0)
  op := inst(15,12)
  imm := inst(31,16)
  
  val isJmp = Bool()
  isJmp := op === UInt(6)
  val isNotJmp = Bool()
  isNotJmp := ~(op === UInt(6))
  when(isNotJmp){
    pcReg := pcPlus4
  }
  val jmpTarget = Bits()
  jmpTarget := imm(3,0)
  when(isJmp){
    pcReg := jmpTarget
  }
  
  val regfile = Mem(Bits(width = 32), 16)

  val rs1Data = UInt()
  val rs2Data = UInt()

  rs1Data := regfile.read(rs1)
  rs2Data := regfile.read(rs2)
  io.readData.bits := regfile.read(io.readAddr.bits)

  io.readAddr.ready := op === Bits(7)
  io.readData.valid := op === Bits(7)

  val operand1 = UInt()
  operand1 := rs1Data
  when(op === UInt(4)){
    operand1 := UInt(0)
  }
  val operand2 = UInt()
  operand2 := rs2Data
  when(op > UInt(1)){
    operand2 := imm
  }
  
  //alu
  val adderOut = UInt()
  val subtractOut = UInt()
  adderOut := operand1 + operand2
  subtractOut := operand1 - operand2
  
  val adderSel = Bool()
  val subtractSel = Bool()
  adderSel := (op === UInt(0)) || (op === UInt(2) || (op === UInt(4)))
  subtractSel := (op === UInt(1)) || (op === UInt(3))
  
  when(adderSel){
    regfile.write(rd, adderOut)
  }
  when(subtractSel){
    regfile.write(rd, subtractOut)
  }

  //mem
  val isLoad = op === Bits(8)
  val isStore = op === Bits(9)
  //mem req setup
  val memWrite = Bits(width = 1)
  memWrite := Bits(0)
  when(isStore){
    memWrite := Bits(1)
  }
  io.dmemPort.req.bits := Cat(memWrite, rs1Data(9, 0), rs2Data)
  io.dmemPort.req.valid := isLoad | isStore

  //mem resp setup
  io.dmemPort.resp.ready := isLoad
  when(isLoad){
    regfile.write(rd, io.dmemPort.resp.bits)
  }

}

class CpuTestBench(outputAddrs: Array[Int], inputData: Array[Int], waitCycles: Int) extends Module {
  val io = new Bundle { 
    val readData = new DecoupledIO(UInt(width = 32)).flip()
    val readAddr = new DecoupledIO(Bits(width = 4))
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }

  val outputState = Reg(init = UInt(0, width = 32))
  val inputState = Reg(init = UInt(0, width = 32))

  io.readData.ready := Bool(false)
  io.readAddr.valid := Bool(false)
  io.readAddr.bits := Bits(0)
  io.passed := Bool(false)
  io.failed := Bool(false)

  for(i <- 0 until waitCycles) {
    when(outputState === UInt(i)){
      outputState := UInt(i + 1)
    }
  }

  for(i <- waitCycles until waitCycles + outputAddrs.length){
    when(outputState === UInt(i)){
      io.readAddr.valid := Bool(true)
      io.readAddr.bits := Bits(outputAddrs(i - waitCycles))
      when(io.readAddr.ready){
        outputState := UInt(i + 1)
      }
    }
  }
  
  //inputData.length is the passed state, inputData.length + 1 is the failed state
  for(i <- 0 until inputData.length){
    when(inputState === UInt(i)){
      io.readData.ready := Bool(true)
      when(io.readData.valid){
        when(io.readData.bits === UInt(inputData(i))){
          inputState := UInt(i + 1)
        }.otherwise {
          inputState := UInt(inputData.length + 1)
        }
      }
    }
  }

  when(inputState === UInt(inputData.length)){//passed state
    io.passed := Bool(true)
  }
  when(inputState === UInt(inputData.length + 1)){
    io.failed := Bool(true)
  }
}

class CpuTestHarness extends Module {
  val io = new Bundle {
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }

  val DUT = Module(new Cpu )
  val ICache = Module(new ICache)
  val DCache = Module(new DCache)
  val testBench = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6), Array(2, 3, 5, 3, 3, 3, 6), 0))
  
  DUT.io.imemPort <> ICache.io
  DUT.io.dmemPort <> DCache.io

  DUT.io.readAddr <> testBench.io.readAddr
  DUT.io.readData <> testBench.io.readData

  io.passed := testBench.io.passed
  io.failed := testBench.io.failed
}

class CpuTests(c: CpuTestHarness) extends Tester(c, Array(c.io)) {
  defTests {
    val vars = new HashMap[Node, Node] ()
    val ovars = new HashMap[Node, Node] ()
    var done = false
    var passed = true
    while(!done){
      //vars(c.io.failed) = Bool(false)
      step(vars, ovars)
      if(ovars(c.io.passed).name == "0x1"){
        done = true
      }
      if(ovars(c.io.failed).name == "0x1"){
        passed = false
        done = true
      }
    }
    passed
  }
}

object CpuProj {
  def main(args: Array[String]): Unit = {
    if(args(0) == "-vbuild"){
      chiselMain(args.slice(1,args.length) ++ Array("--backend", "v"), () => Module(new Cpu()))
    } else if(args(0) == "-backannotation"){
      chiselMain(args.slice(1,args.length) ++ Array("--backend", "MyBackend.MyBackend"), () => Module(new Cpu()))
    } else {
      chiselMainTest(args, () => Module(new CpuTestHarness())) {
        c => new CpuTests(c)
      }
    }
  }
}
