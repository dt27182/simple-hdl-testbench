package Cpu 
 
import Chisel._

class TICacheIo extends Bundle
{
  val req = new ValidIO(Bits(width=4)).flip()
  val resp = Bits(OUTPUT, 32)
}

class TICache() extends Module
{
  val io = new TICacheIo()
  val num_lines = 16
  val imem = Vec.fill(num_lines){Bits(width = 32)}


  //fill in instructions
  //instruction format:
  //31-16:Imm | 15-12:Op | 11-8:rs1 | 7-4:rs2 | 3-0: rd 
  //op = 0 => add
  //op = 1 => sub
  //op = 2 => addi
  //op = 3 => subi
  //op = 4 => seti
  //op = 5 => nop
  //op = 6 => jmp
  //op = 7 => service exteral reg read
  //op = 8 => load
  //op = 9 => store
  imem(0) := Cat(Bits(1,16), Bits(4,4), Bits(0,4), Bits(0,4), Bits(0,4)) //r0 = 1
  imem(1) := Cat(Bits(1,16), Bits(4,4), Bits(1,4), Bits(0,4), Bits(1,4)) //r1 = 1
  imem(2) := Cat(Bits(1,16), Bits(2,4), Bits(0,4), Bits(15,4), Bits(0,4)) //r0 = r0 + 1
  imem(3) := Cat(Bits(2,16), Bits(2,4), Bits(1,4), Bits(15,4), Bits(1,4)) //r1 = r1 + 2
  imem(4) := Cat(Bits(0,16), Bits(0,4), Bits(0,4), Bits(1,4), Bits(2,4)) //r2 = r0 + r1
  imem(5) := Cat(Bits(0,16), Bits(1,4), Bits(2,4), Bits(0,4), Bits(3,4)) //r3 = r2 - r0
  imem(6) := Cat(Bits(1,16), Bits(4,4), Bits(4,4), Bits(15,4), Bits(4,4)) //r4 = 1
  imem(7) := Cat(Bits(9,16), Bits(6,4), Bits(0,4), Bits(15,4), Bits(0,4)) //jump to PC=9
  imem(8) := Cat(Bits(1,16), Bits(2,4), Bits(4,4), Bits(15,4), Bits(4,4)) //r4 = r4 + 1
  imem(9) := Cat(Bits(2,16), Bits(2,4), Bits(4,4), Bits(15,4), Bits(4,4)) //r4 = r4 + 2
  imem(10) := Cat(Bits(0,16), Bits(9,4), Bits(0,4), Bits(4,4), Bits(4,4)) //M(r4) = r4
  imem(11) := Cat(Bits(0,16), Bits(8,4), Bits(0,4), Bits(4,4), Bits(5,4)) //r5 = M(r4)
  imem(12) := Cat(Bits(0,16), Bits(0,4), Bits(5,4), Bits(3,4), Bits(6,4)) //r6 = r3 + r5
  imem(13) := Cat(Bits(0,16), Bits(5,4), Bits(0,4), Bits(15,4), Bits(0,4)) //nop
  imem(14) := Cat(Bits(0,16), Bits(7,4), Bits(0,4), Bits(15,4), Bits(0,4)) //service exteral reg read
  imem(15) := Cat(Bits(14,16), Bits(6,4), Bits(0,4), Bits(15,4), Bits(0,4)) //jump to PC = 14(stop execution here)
  
  //override val req_ready = Bool(true)
  val mem_addr = io.req.bits
  //acceptBackPressure = false
  io.resp := imem(mem_addr)
  //override val resp_valid = Bool(true)

}
/*
package Hello 
 
import Chisel._

class TICacheIo extends TransactionalBundle
{
  override val req = new ValidIO(Bits(width=4)).flip()
  override val resp = Bits(OUTPUT, 32)
}

class TICache() extends TransactionalComponent
{
  val io = new TICacheIo()
  val num_lines = 16
  val imem = Mem(Bits(width = 32), num_lines)

  override val req_ready = Bool(true)
  val mem_addr = io.req.bits
  acceptBackPressure = false
  io.resp := imem(mem_addr)
  override val resp_valid = Bool(true)

}*/
