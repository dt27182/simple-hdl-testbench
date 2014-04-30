run: copy-fsm-src
	sbt "project fsm" "run --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd FSM.vcd FSM.vcd.vpd

copy-fsm-src: gen-src
	cp ../simple-hdl-proj/generated/fsm.scala src/fsm/.

gen-src:
	cd ../simple-hdl-proj && sbt "project fsm" "run"

run-cpu: src/cpu/cpu-dut.scala
	sbt "project cpu" "run --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd CpuTestHarness.vcd CpuTestHarness.vcd.vpd

src/cpu/cpu-dut.scala: ../simple-hdl-proj/generated/cpu-dut.scala
	cp ../simple-hdl-proj/generated/cpu-dut.scala src/cpu/.

../simple-hdl-proj/generated/cpu-dut.scala: ../simple-hdl-proj/src/cpu/cpu.scala ../simple-hdl-proj/src
	cd ../simple-hdl-proj && sbt "project cpu" "run"

verilog:
	sbt "project fsm" "run --backend Chisel.Fame5VerilogBackend --targetDir generated"

clean:
	sbt "project chisel" clean
	sbt "project cpu" clean
	rm -f emulator/*.cpp
	rm -f emulator/*.h
	rm -f emulator/*.o
	rm -rf target
	rm -f emulator/Hello
	rm *.vpd
	rm *.vcd
