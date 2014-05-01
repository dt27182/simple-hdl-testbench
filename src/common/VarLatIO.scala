package Common

import Chisel._

class VarLatIO(req_width:Int, resp_width:Int) extends Bundle {
  val reqBits = Bits(OUTPUT, width = req_width)
  val reqValid = Bool(OUTPUT)
  val respBits = Bits(INPUT, width = resp_width)
  val respPending = Bool(INPUT)
}
