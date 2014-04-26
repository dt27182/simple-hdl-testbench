package Cpu

import Chisel._
import scala.collection.mutable.HashMap
import Common._

class DCache extends Module {
  val io = new VarLatIO(43,32).flip()

  val mem = Mem(Bits(width = 32), 1024)

  val doWrite = io.req.bits(42)
  val reqAddr = io.req.bits(41, 32)
  val writeData = io.req.bits(31, 0)

  when(io.req.valid && doWrite){
    mem.write(reqAddr, writeData)
  }

  io.resp.bits := mem.read(reqAddr)
  io.resp.valid := io.req.valid && ~doWrite

  io.req.ready := Bool(true)
  io.respPending := Bool(false)
}
