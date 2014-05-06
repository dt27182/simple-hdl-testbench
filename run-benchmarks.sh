for i in `seq 1 4`;
do
  for j in `seq 1 4`;
  do
    for k in `seq 1 1`;
    do
      for l in `seq 0 8`;
      do
        cd ~/multithread-transform/simple-hdl-proj
        sbt "project cpu" "run $i $j $k"
        cp generated/Cpu.scala ~/multithread-transform/simple-hdl-testbench/src/cpu/Cpu.scala
        cd ~/multithread-transform/simple-hdl-testbench
        python src/cpu/gen-testharness.py $i
        sbt "project cpu" "run -ctest $i $l --backend c --genHarness --compile --test --vcd --debug --targetDir emulator" > reports/Cpu$i$j$k$l.rpt
        echo $i $j $k $l
      done
    done
  done
done

for i in `seq 1 4`;
do
  for j in `seq 1 $i`;
  do
    for k in `seq 0 0`;
    do
      for l in `seq 0 8`;
      do
        cd ~/multithread-transform/simple-hdl-proj
        sbt "project cpu" "run $i $j $k"
        cp generated/Cpu.scala ~/multithread-transform/simple-hdl-testbench/src/cpu/Cpu.scala
        cd ~/multithread-transform/simple-hdl-testbench
        python src/cpu/gen-testharness.py $i
        sbt "project cpu" "run -ctest $i $l --backend c --genHarness --compile --test --vcd --debug --targetDir emulator" > reports/Cpu$i$j$k$l.rpt
        echo $i $j $k $l
      done
    done
  done
done
