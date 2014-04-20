package Cpu 
 
import Chisel._ 
import scala.collection.mutable.HashMap

class Cpu extends Module {
  val io = new Bundle { 
    val read_data = new DecoupledIO(UInt(width = 32))
    val read_addr = new DecoupledIO(Bits(width = 4)).flip()
  }

  val pc_reg = Reg(init = UInt(0,4))
  val pc_spec = UInt()
  pc_spec := pc_reg + UInt(1)
  val pc_plus4 = UInt()
  pc_plus4 := pc_reg + UInt(1)
  
  val imem = Module(new TICache())
  imem.io.req.valid := Bool(true)
  val imem_addr = Bits()
  imem_addr := pc_reg
  imem.io.req.bits := imem_addr
  
  val inst = Bits()
  inst := imem.io.resp

  val rs1 = Bits()
  val rs2 = Bits()
  val rd = Bits()
  val op = Bits()
  val imm = Bits()
  rs1 := inst(11,8)
  rs2 := inst(7,4)
  rd := inst(3,0)
  op := inst(15,12)
  imm := Cat(Bits(0,16), inst(31,16))
  
  val isJmp = Bool()
  isJmp := op === UInt(6)
  val isNotJmp = Bool()
  isNotJmp := ~(op === UInt(6))
  when(isNotJmp){
    pc_reg := pc_plus4
  }
  val jmpTarget = Bits()
  jmpTarget := imm(3,0)
  when(isJmp){
    pc_reg := jmpTarget
  }
  
  //val regfile = Module(new TransactionMem(16, 3, 3, Array(0,1,2), 2, 1, Array(0, 0))(Bits(width = 32)))
  val regfile = Mem(Bits(width = 32), 16)

  val rs1_data = UInt()
  val rs2_data = UInt()

  rs1_data := regfile.read(rs1)
  rs2_data := regfile.read(rs2)
  io.read_data.bits := regfile.read(io.read_addr.bits)

  io.read_addr.ready := op === Bits(7)
  io.read_data.valid := op === Bits(7)

  val operand1 = UInt()
  operand1 := rs1_data
  when(op === UInt(4)){
    operand1 := UInt(0)
  }
  val operand2 = UInt()
  operand2 := rs2_data
  when(op > UInt(1)){
    operand2 := imm
  }
  
  val adder_out = UInt()
  val subtract_out = UInt()
  adder_out := operand1 + operand2
  subtract_out := operand1 - operand2
  
  val adder_sel = Bool()
  val subtract_sel = Bool()
  adder_sel := (op === UInt(0)) || (op === UInt(2) || (op === UInt(4)))
  subtract_sel := (op === UInt(1)) || (op === UInt(3))
  
  when(adder_sel){
    regfile.write(rd, adder_out)
  }
  when(subtract_sel){
    regfile.write(rd, subtract_out)
  }
}

class CpuTestBench(outputAddrs: Array[Int], inputData: Array[Int], waitCycles: Int) extends Module {
  val io = new Bundle { 
    val read_data = new DecoupledIO(UInt(width = 32)).flip()
    val read_addr = new DecoupledIO(Bits(width = 4))
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }

  val outputState = Reg(init = UInt(0, width = 32))
  val inputState = Reg(init = UInt(0, width = 32))

  io.read_data.ready := Bool(false)
  io.read_addr.valid := Bool(false)
  io.read_addr.bits := Bits(0)
  io.passed := Bool(false)
  io.failed := Bool(false)

  for(i <- 0 until waitCycles) {
    when(outputState === UInt(i)){
      outputState := UInt(i + 1)
    }
  }

  for(i <- waitCycles until waitCycles + outputAddrs.length){
    when(outputState === UInt(i)){
      io.read_addr.valid := Bool(true)
      io.read_addr.bits := Bits(outputAddrs(i - waitCycles))
      when(io.read_addr.ready){
        outputState := UInt(i + 1)
      }
    }
  }
  
  //inputData.length is the passed state, inputData.length + 1 is the failed state
  for(i <- 0 until inputData.length){
    when(inputState === UInt(i)){
      io.read_data.ready := Bool(true)
      when(io.read_data.valid){
        when(io.read_data.bits === UInt(inputData(i))){
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
  val testBench = Module(new CpuTestBench(Array(0, 1, 2, 3, 4), Array(2, 3, 5, 3, 3), 0))

  DUT.io.read_addr <> testBench.io.read_addr
  DUT.io.read_data <> testBench.io.read_data

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

object Cpu {
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
