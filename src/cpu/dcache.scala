package Cpu

import Chisel._
import scala.collection.mutable.HashMap
import Common._

/*class DCache(missLatency: Int = 0, lfsrWait: Int = 0) extends Module {
  val io = new VarLatIO(43,32).flip()

  val doWrite = io.reqBits(42)
  val reqAddr = io.reqBits(41, 32)
  val writeData = io.reqBits(31, 0)

  val idle :: miss :: Nil = Enum(UInt(), 2)
  val currentState = Reg(init = idle)
  val lastMissAddr = Reg(init = UInt(0, 32))
  val missCounter = Reg(init = UInt(0, 32))
  
  val lfsrWaitCounter = Reg(init = UInt(0, 32))
  when(lfsrWaitCounter < UInt(lfsrWait)){
    lfsrWaitCounter := lfsrWaitCounter + UInt(1)
  }
  val lfsr = LFSR16(lfsrWaitCounter === UInt(lfsrWait))
  
  io.respPending := Bool(false)
  when(currentState === idle){
    when(io.reqValid && lfsr(0).toBool && reqAddr != lastMissAddr){
      io.respPending := Bool(true)
      currentState := miss
      lastMissAddr := reqAddr
    }
  } .elsewhen(currentState === miss){
    io.respPending := Bool(true)
    when(missCounter === UInt(missLatency)){
      currentState := idle
      missCounter := UInt(0)
    }.otherwise{
      missCounter := missCounter + UInt(1)
    }
  }

  val mem = Mem(Bits(width = 32), 1024)
    when(io.reqValid && doWrite && currentState === idle){
    mem.write(reqAddr, writeData)
  }

  io.respBits := mem.read(reqAddr)
}*/

class DCache(missLatency: Int = 0, lfsrWait: Int = 0) extends Module {
  val io = new VarLatIO(43,32).flip()

  val doWrite = io.reqBits(42)
  val reqAddr = io.reqBits(41, 32)
  val writeData = io.reqBits(31, 0)

  io.respPending := Bool(false)

  val mem = Mem(Bits(width = 32), 1024)
  when(io.reqValid && doWrite){
    mem.write(reqAddr, writeData)
  }

  io.respBits := mem.read(reqAddr)
}
