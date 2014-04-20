package plustwo 
 
import Chisel._ 
import scala.collection.mutable.HashMap

class PlusOne extends Module {
  val io = new Bundle {
    val in = UInt(INPUT)
    val out = UInt(OUTPUT)
  }
  io.out := io.in + UInt(1)
}

class DUT extends Module {
  val io = new Bundle {
    val in = UInt(INPUT, width = 32)
    val out = UInt(OUTPUT, width = 32)
  }
  io.out := io.in + UInt(1)
}

class DUTTests(c: DUT) extends Tester(c) {
  for(i <- 0 until 4){
    val input = rnd.nextInt(32)
    poke(c.io.in, input)
    step(1)
    expect(c.io.out, input + 2)
  }
}

object PlusTwo {
  def main(args: Array[String]): Unit = {
    chiselMainTest(args ++ Array("--backend", "plustwo.PlusCppBackend"), () => Module(new DUT())) {
      c => new DUTTests(c)
    }
  }
}
