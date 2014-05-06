package Cpu 
import Chisel._ 
import scala.collection.mutable.HashMap
import Common._
import scala.collection.mutable.ArrayBuffer

class CpuTestHarness(numThreads:Int = 1, delay: Int = 0) extends Module {
  val io = new Bundle {
    val passed = Bool(OUTPUT)
    val failed = Bool(OUTPUT)
  }

  val DUT = Module(new Cpu )
  val testBenches = new ArrayBuffer[CpuTestBench]
  val icaches = new ArrayBuffer[ICache]
  val dcaches = new ArrayBuffer[DCache]
  
  for(i <- 0 until numThreads){
    icaches += Module(new ICache(delay, i%5))
    dcaches += Module(new DCache(delay, i%6))
    if(i%2 == 0){
      testBenches += Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6), Array(2, 3, 5, 3, 3, 3, 6), 0))
    } else {
      testBenches += Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6).reverse, Array(2, 3, 5, 3, 3, 3, 6).reverse, 0))
    }
  }
  DUT.io.imemPort_0 <> icaches(0).io
  DUT.io.dmemPort_0 <> dcaches(0).io
  DUT.io.readAddr_0 <> testBenches(0).io.readAddr
  DUT.io.readData_0 <> testBenches(0).io.readData
  var passed = Bool(true)
  for(i <- 0 until numThreads){
    passed = passed && testBenches(0).io.passed
  }
  var failed = Bool(false)
  for(i <- 0 until numThreads){
    failed = failed || testBenches(0).io.failed
  }
  io.passed := passed
  io.failed := failed
}
