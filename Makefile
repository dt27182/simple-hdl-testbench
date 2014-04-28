run: copy-fsm-src
	sbt "project fsm" "run --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd FSM.vcd FSM.vcd.vpd

run-cpu: copy-cpu-src
	sbt "project cpu" "run --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd CpuTestHarness.vcd CpuTestHarness.vcd.vpd


copy-fsm-src: gen-src
	cp ../simple-hdl-proj/generated/fsm.scala src/fsm/.

copy-cpu-src: gen-src-cpu
	cp ../simple-hdl-proj/generated/cpu-dut.scala src/cpu/.

gen-src:
	cd ../simple-hdl-proj && sbt "project fsm" "run"

gen-src-cpu:
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
