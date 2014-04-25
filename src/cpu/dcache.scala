package Cpu

import Chisel._
import scala.collection.mutable.HashMap
import Common._

class DCache extends Module {
  val io = new Bundle {
    val varLatIO = new VarLatIO(43,32).flip()
  }

  val mem = Mem(Bits(width = 32), 1024)

  val doWrite = io.varLatIO.req.bits(42)
  val reqAddr = io.varLatIO.req.bits(41, 32)
  val writeData = io.varLatIO.req.bits(31, 0)

  when(io.varLatIO.req.valid && doWrite){
    mem.write(reqAddr, writeData)
  }

  io.varLatIO.resp.bits := mem.read(reqAddr)
  io.varLatIO.resp.valid := io.varLatIO.req.valid && ~doWrite

  io.varLatIO.req.ready := Bool(true)
  io.varLatIO.respPending := Bool(false)
}
