package Cpu

import Chisel._
import scala.collection.mutable.HashMap
import Common._

class DCache extends Module {
  val io = new VarLatIO(43,32).flip()

  val mem = Mem(Bits(width = 32), 1024)

  val doWrite = io.reqBits(42)
  val reqAddr = io.reqBits(41, 32)
  val writeData = io.reqBits(31, 0)

  when(io.reqValid && doWrite){
    mem.write(reqAddr, writeData)
  }

  io.respBits := mem.read(reqAddr)
  io.respPending := Bool(false)
}
