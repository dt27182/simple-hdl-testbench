import os
import re
import sys
numThreads = sys.argv[1]

fileName = "/home/eecs/wenyu/multithread-transform/simple-hdl-testbench/src/cpu/testharness.scala"

outputFile = open(fileName, "w")

outputFile.write("package Cpu \n")
outputFile.write("import Chisel._ \n")
outputFile.write("import scala.collection.mutable.HashMap\n")
outputFile.write("import Common._\n")
outputFile.write("import scala.collection.mutable.ArrayBuffer\n")
outputFile.write("\n")
outputFile.write("class CpuTestHarness(numThreads:Int = 1, delay: Int = 0) extends Module {\n")
outputFile.write("  val io = new Bundle {\n")
outputFile.write("    val passed = Bool(OUTPUT)\n")
outputFile.write("    val failed = Bool(OUTPUT)\n")
outputFile.write("  }\n")
outputFile.write("\n")
outputFile.write("  val DUT = Module(new Cpu )\n")
outputFile.write("  val testBenches = new ArrayBuffer[CpuTestBench]\n")
outputFile.write("  val icaches = new ArrayBuffer[ICache]\n")
outputFile.write("  val dcaches = new ArrayBuffer[DCache]\n")
outputFile.write("  \n")
outputFile.write("  for(i <- 0 until numThreads){\n")
outputFile.write("    icaches += Module(new ICache(delay, i%5))\n")
outputFile.write("    dcaches += Module(new DCache(delay, i%6))\n")
outputFile.write("    if(i%2 == 0){\n")
outputFile.write("      testBenches += Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6), Array(2, 3, 5, 3, 3, 3, 6), 0))\n")
outputFile.write("    } else {\n")
outputFile.write("      testBenches += Module(new CpuTestBench(Array(0, 1, 2, 3, 4, 5, 6).reverse, Array(2, 3, 5, 3, 3, 3, 6).reverse, 0))\n")
outputFile.write("    }\n")
outputFile.write("  }\n")

for i in range(0, int(numThreads)):
  outputFile.write("  DUT.io.imemPort_" + str(i) + " <> icaches(" + str(i) + ").io\n")
  outputFile.write("  DUT.io.dmemPort_" + str(i) + " <> dcaches(" + str(i) + ").io\n")
  outputFile.write("  DUT.io.readAddr_" + str(i) + " <> testBenches(" + str(i) + ").io.readAddr\n")
  outputFile.write("  DUT.io.readData_" + str(i) + " <> testBenches(" + str(i) + ").io.readData\n")

outputFile.write("  var passed = Bool(true)\n")
outputFile.write("  for(i <- 0 until numThreads){\n")
outputFile.write("    passed = passed && testBenches(" + str(i) + ").io.passed\n")
outputFile.write("  }\n")
outputFile.write("  var failed = Bool(false)\n") 
outputFile.write("  for(i <- 0 until numThreads){\n")
outputFile.write("    failed = failed || testBenches(" + str(i) + ").io.failed\n")
outputFile.write("  }\n")

outputFile.write("  io.passed := passed\n")
outputFile.write("  io.failed := failed\n")
outputFile.write("}\n")
outputFile.close
