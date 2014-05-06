run: copy-fsm-src
	sbt "project fsm" "run --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd FSM.vcd FSM.vcd.vpd

copy-fsm-src: gen-src
	cp ../simple-hdl-proj/generated/fsm.scala src/fsm/.

gen-src:
	cd ../simple-hdl-proj && sbt "project fsm" "run"

run-cpu: copy-cpu-src
	sbt "project cpu" "run -ctest 4 20 --backend c --genHarness --compile --test --vcd --debug --targetDir emulator";vcd2vpd CpuTestHarness.vcd CpuTestHarness.vcd.vpd

copy-cpu-src: generate-cpu-src 
	cp ../simple-hdl-proj/generated/Cpu.scala src/cpu/.

generate-cpu-src: ../simple-hdl-proj/src/cpu/cpu.scala ../simple-hdl-proj/src
	cd ../simple-hdl-proj && sbt "project cpu" "run 4 4 1"

verilog:
	sbt "project cpu" "run -vbuild --targetDir generated"

clean:
	sbt "project chisel" clean
	sbt "project cpu" clean
	rm -f emulator/*.cpp
	rm -f emulator/*.h
	rm -f emulator/*.o
	rm -rf target
	rm -f emulator/Hello
	rm -f *.vpd
	rm -f *.vcd
	rm -f generated/*.v
	
