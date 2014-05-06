package Cpu 
 
import Chisel._ 
import scala.collection.mutable.HashMap
import Common._
import scala.collection.mutable.ArrayBuffer

/*
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

}*/

class CpuTestBench(outputAddrs: Array[Int], inputData: Array[Int], waitCycles: Int) extends Module {
  val io = new Bundle { 
    val readData = new DecoupledIO(UInt(width = 32)).flip()
    val readAddr = new DecoupledIO(Bits(width = 4))
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }
  
  val fire = Bool()
  val lfsr = LFSR16()
  fire := lfsr(0).toBool()
  
  val outputState = Reg(init = UInt(0, width = 32))
  val inputState = Reg(init = UInt(0, width = 32))

  io.readData.ready := Bool(false)
  io.readAddr.valid := Bool(false)
  io.readAddr.bits := Bits(0)
  io.passed := Bool(false)
  io.failed := Bool(false)

  for(i <- 0 until waitCycles) {
    //when(outputState === UInt(i) && fire){
    when(outputState === UInt(i)){
      outputState := UInt(i + 1)
    }
  }

  for(i <- waitCycles until waitCycles + outputAddrs.length){
    //when(outputState === UInt(i) && fire){
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
    //when(inputState === UInt(i) && fire){
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
    io.readData.ready := Bool(true)
    io.readAddr.valid := Bool(true)
  }
  when(inputState === UInt(inputData.length + 1)){
    io.failed := Bool(true)
  }
}

//single thread testharness

/*
class CpuTestHarness extends Module {
  val io = new Bundle {
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }

  val DUT = Module(new Cpu )
  val ICache = Module(new ICache(10, 4))
  val DCache = Module(new DCache(10, 10))
  val testBench = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6), Array(2, 3, 5, 3, 3, 3, 6), 0))
  
  DUT.io.imemPort_0 <> ICache.io
  DUT.io.dmemPort_0 <> DCache.io

  DUT.io.readAddr_0 <> testBench.io.readAddr
  DUT.io.readData_0 <> testBench.io.readData

  io.passed := testBench.io.passed
  io.failed := testBench.io.failed
}*/

//multi-thread testharness with 2 separate cpus
/*class CpuTestHarness extends Module {
  val io = new Bundle {
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }

  val DUT0 = Module(new Cpu )
  val ICache0 = Module(new ICache(10, 4))
  val DCache0 = Module(new DCache)
  val testBench0 = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6), Array(2, 3, 5, 3, 3, 3, 6), 0))
  
  DUT0.io.imemPort <> ICache0.io
  DUT0.io.dmemPort <> DCache0.io

  DUT0.io.readAddr <> testBench0.io.readAddr
  DUT0.io.readData <> testBench0.io.readData

  val DUT1 = Module(new Cpu )
  val ICache1 = Module(new ICache(10, 4))
  val DCache1 = Module(new DCache)
  val testBench1 = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6).reverse, Array(2, 3, 5, 3, 3, 3, 6).reverse, 0))
  
  DUT1.io.imemPort <> ICache1.io
  DUT1.io.dmemPort <> DCache1.io

  DUT1.io.readAddr <> testBench1.io.readAddr
  DUT1.io.readData <> testBench1.io.readData

  io.passed := testBench0.io.passed && testBench1.io.passed
  io.failed := testBench0.io.failed || testBench1.io.failed
}*/

//multithread test harness with 1 2thread cpu
/*class CpuTestHarness extends Module {
  val io = new Bundle {
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }

  val DUT = Module(new Cpu )
  val ICache0 = Module(new ICache(10, 7))
  val DCache0 = Module(new DCache(10, 7))
  val testBench0 = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6), Array(2, 3, 5, 3, 3, 3, 6), 0))
  
  DUT.io.imemPort_0 <> ICache0.io
  DUT.io.dmemPort_0 <> DCache0.io

  DUT.io.readAddr_0 <> testBench0.io.readAddr
  DUT.io.readData_0 <> testBench0.io.readData

  val ICache1 = Module(new ICache(10, 4))
  val DCache1 = Module(new DCache(10, 4))
  val testBench1 = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6).reverse, Array(2, 3, 5, 3, 3, 3, 6).reverse, 0))
  
  DUT.io.imemPort_1 <> ICache1.io
  DUT.io.dmemPort_1 <> DCache1.io

  DUT.io.readAddr_1 <> testBench1.io.readAddr
  DUT.io.readData_1 <> testBench1.io.readData

  val ICache2 = Module(new ICache(10, 8))
  val DCache2 = Module(new DCache(10, 8))
  val testBench2 = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6), Array(2, 3, 5, 3, 3, 3, 6), 0))
  
  DUT.io.imemPort_2 <> ICache2.io
  DUT.io.dmemPort_2 <> DCache2.io

  DUT.io.readAddr_2 <> testBench2.io.readAddr
  DUT.io.readData_2 <> testBench2.io.readData


  val ICache3 = Module(new ICache(10, 3))
  val DCache3 = Module(new DCache(10, 3))
  val testBench3 = Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6).reverse, Array(2, 3, 5, 3, 3, 3, 6).reverse, 0))
  
  DUT.io.imemPort_3 <> ICache3.io
  DUT.io.dmemPort_3 <> DCache3.io

  DUT.io.readAddr_3 <> testBench3.io.readAddr
  DUT.io.readData_3 <> testBench3.io.readData

  io.passed := testBench0.io.passed && testBench1.io.passed && testBench2.io.passed && testBench3.io.passed
  io.failed := testBench0.io.failed || testBench1.io.failed || testBench2.io.failed || testBench3.io.failed

  //io.passed := testBench0.io.passed && testBench1.io.passed
  //io.failed := testBench0.io.failed || testBench1.io.failed
}*/

class CpuTests(c: CpuTestHarness) extends Tester(c, Array(c.io)) {
  defTests {
    val vars = new HashMap[Node, Node] ()
    val ovars = new HashMap[Node, Node] ()
    var done = false
    var passed = true
    var cycles = 0
    while(!done){
      //vars(c.io.failed) = Bool(false)
      step(vars, ovars)
      cycles = cycles + 1
      if(ovars(c.io.passed).name == "0x1"){
        done = true
      }
      if(ovars(c.io.failed).name == "0x1"){
        passed = false
        done = true
      }
    }
    println("Cycle Count: " + cycles)
    passed
  }
}

object CpuProj {
  def main(args: Array[String]): Unit = {
    if(args(0) == "-vbuild"){
      chiselMain(args.slice(1,args.length) ++ Array("--backend", "v"), () => Module(new Cpu()))
    } else if(args(0) == "-backannotation"){
      chiselMain(args.slice(1,args.length) ++ Array("--backend", "MyBackend.MyBackend"), () => Module(new Cpu()))
    } else if(args(0) == "-ctest"){
      chiselMainTest(args.slice(3, args.length), () => Module(new CpuTestHarness(args(1).toInt, args(2).toInt))) {
        c => new CpuTests(c)
      }
    } else {
      chiselMainTest(args, () => Module(new CpuTestHarness(4, 10))) {
        c => new CpuTests(c)
      }
    }
  }
}
