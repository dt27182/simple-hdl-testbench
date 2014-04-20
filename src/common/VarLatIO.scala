package Common

import Chisel._

class VarLatIO(req_width:Int, resp_width:Int) extends Bundle {
  val req = new DecoupledIO(Bits(width=req_width))
  val resp = new DecoupledIO(Bits(width=resp_width)).flip()
  val respPending = Bool(INPUT)
}
