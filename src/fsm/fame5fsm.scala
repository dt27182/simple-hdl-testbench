package FSM 
 
import Chisel._ 
import Common._
import scala.collection.mutable.HashMap


class TransactionMemTester(addrs: Array[Int], datas: Array[Int]) extends Module {
  val io = new Bundle{
    val addr_out = new DecoupledIO(Bits(width = 2))
    val data_in = new DecoupledIO(UInt(width = 32)).flip
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }
  //target machine
  val send0 :: send1 :: check0 :: check1 :: passed :: failed :: Nil = Enum(UInt(), 6)
  val currentState = Reg(init = send0)
  val nextState = UInt()
  currentState := nextState

  nextState:= currentState
  io.addr_out.valid := Bool(false)
  io.addr_out.bits := UInt(0)
  io.data_in.ready := Bool(false)
  io.passed := Bool(false)
  io.failed := Bool(false)
  when(currentState === send0){
    io.addr_out.valid := Bool(true)
    io.addr_out.bits := UInt(addrs(0))
    when(io.addr_out.ready){
      nextState := send1
    }
  }.elsewhen(currentState === send1){
    io.addr_out.valid := Bool(true)
    io.addr_out.bits := UInt(addrs(1))
    when(io.addr_out.ready){
      nextState := check0
    }
  }.elsewhen(currentState === check0){
    io.data_in.ready := Bool(true)
    when(io.data_in.valid){
      when(io.data_in.bits === UInt(datas(0))){
        nextState := check1
      }.otherwise{
        nextState := failed
      }
    }
  }.elsewhen(currentState === check1){
    io.data_in.ready := Bool(true)
    when(io.data_in.valid){
      when(io.data_in.bits === UInt(datas(1))){
        nextState := passed
      }.otherwise{
        nextState := failed
      }
    }
  }.elsewhen(currentState === passed){
    io.passed := Bool(true)
  }.elsewhen(currentState === failed){
    io.failed := Bool(true)
  }
}

class FSM extends Module {
  val io = new Bundle {
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }
  val DUT = Module(new TransactionMemDUT)
  val tester = Module(new TransactionMemTester(Array(2,3), Array(1,0)))

  val addrQueue = Module(new Queue(Bits(width = 2), 8))
  val dataQueue = Module(new Queue(UInt(width = 32), 8))

  addrQueue.io.enq <> tester.io.addr_out
  addrQueue.io.deq <> DUT.io.addr_in
  dataQueue.io.enq <> DUT.io.data_out
  dataQueue.io.deq <> tester.io.data_in

  io.passed := tester.io.passed
  io.failed := tester.io.failed
}

/*
class FSMTests(c: FSM) extends Tester(c) {
  poke(c.io.in, 1)
  expect(c.io.out, 0)
  step(1)

  poke(c.io.in, 2)
  expect(c.io.out, 0)
  step(1)

  poke(c.io.in, 3)
  expect(c.io.out, 1)
  step(1)

  expect(c.io.out, 3)
  step(1)

  expect(c.io.out, 6)
}*/


class FSMTests(c: FSM) extends Tester(c, Array(c.io)) {
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

object test {
  def main(args: Array[String]): Unit = {
    if(args(0) == "-ctest"){
      chiselMainTest(args.slice(1, args.length) ++ Array("--backend", "c", "--genHarness", "--compile", "--test", "--vcd", "--debug"), () => Module(new FSM())) {
        c => new FSMTests(c) }
    } else if(args(0) == "-vbuild"){
      chiselMain(args.slice(1,args.length), () => Module(new FSM()))
    } else if(args(0) == "-backannotation"){
      chiselMain(args.slice(1,args.length) ++ Array("--backend", "MyBackend.MyBackend"), () => Module(new FSM()))
    } else {
      chiselMainTest(args, () => Module(new FSM())) {
        c => new FSMTests(c)
      }
    }
  }
}
