#for j in `seq 1 4`;
#do
#  for i in `seq 1 4`;
#  do
#    for k in `seq 1 1`;
#    do
#      scp wenyu@s141.millennium.berkeley.edu:~/multithread-transform/simple-hdl-testbench/generated/Cpu$i$j$k.v ~/multithread-transform/simple-hdl-testbench/vlsi/build/generated-src/Cpu.v
#      make
#      cp -R current-dc/reports reports$i$j$k
#      echo $i $j $k
#    done
#  done
#done

for i in `seq 3 3`;
do
  for j in `seq 1 1`;
  do
    for k in `seq 0 0`;
    do
      scp wenyu@s141.millennium.berkeley.edu:~/multithread-transform/simple-hdl-testbench/generated/Cpu$i$j$k.v ~/multithread-transform/simple-hdl-testbench/vlsi/build/generated-src/Cpu.v
      make
      cp -R current-dc/reports reports$i$j$k
      echo $i $j $k
    done
  done
done
