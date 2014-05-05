for i in `seq 1 4`;
do
  for j in `seq 1 4`;
  do
    for k in `seq 1 1`;
    do
      cd ~/multithread-transform/simple-hdl-proj
      sbt "project cpu" "run $i $j $k"
      cp generated/Cpu.scala ~/multithread-transform/simple-hdl-testbench/src/cpu/Cpu.scala
      cd ~/multithread-transform/simple-hdl-testbench
      make verilog
      mv generated/Cpu.v generated/Cpu$i$j$k.v
    done
  done
done

for i in `seq 1 4`;
do
  for j in `seq $i 4`;
  do
    for k in `seq 0 0`;
    do
      cd ~/multithread-transform/simple-hdl-proj
      sbt "project cpu" "run $i $j $k"
      cp generated/Cpu.scala ~/multithread-transform/simple-hdl-testbench/src/cpu/Cpu.scala
      cd ~/multithread-transform/simple-hdl-testbench
      make verilog
      mv generated/Cpu.v generated/Cpu$i$j$k.v
    done
  done
done
