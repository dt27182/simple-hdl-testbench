run: copy-src
	sbt "project fsm" "run --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd FSM.vcd FSM.vcd.vpd

run-cpu: 
	sbt "project cpu" "run --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd CpuTestHarness.vcd CpuTestHarness.vcd.vpd


copy-src: gen-src
	cp ../simple-hdl-proj/generated/*.scala src/fsm/.

gen-src:
	cd ../simple-hdl-proj && sbt "project fsm" "run"

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
